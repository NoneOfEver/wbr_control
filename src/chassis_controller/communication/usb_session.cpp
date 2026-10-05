/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/chassis_controller/communication/usb_session.cpp
 * @ingroup wbr_platform
 * @brief 管理 USB 通信会话及其收发状态。
 * @details 实现封装 Zephyr 设备 API、硬件初始化和异步回调。共享状态在中断、回调与线程之间访问时使用原子量、内核队列或短临界区保护。
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <msg/booster_state.hpp>
#include <msg/gimbal_state.hpp>
#include <msg/hi91_imu_sample.hpp>
#include <msg/pc_auto_aim_command.hpp>
#include <msg/remote_input_state.hpp>
#include <msg/usb_raw_frame_queue.hpp>
#include <chassis_controller/communication/usb_session.h>
#include <chassis_controller/scheduling/thread_phases.h>
#include <chassis_controller/scheduling/thread_priorities.h>
#include <protocols/pc_link/pc_link.h>
#include <periodic_schedule.h>

#if defined(CONFIG_WBR_CONTROL_USB_MPC) && CONFIG_WBR_CONTROL_USB_MPC
#include <chassis_controller/modules/chassis/ethercat_chassis_port.h>
#include <protocols/mpc_link/mpc_link.h>
#endif

#include "usbd_core.h"

LOG_MODULE_REGISTER(usb_session, LOG_LEVEL_INF);

namespace {

#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE

constexpr uint8_t kBusId = 0U;

#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb0))
constexpr uintptr_t kUsbRegBase = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));
constexpr bool kHasUsbNode = true;
#else
constexpr uintptr_t kUsbRegBase = 0U;
constexpr bool kHasUsbNode = false;
#endif

constexpr uint8_t kVendorInEp = 0x81U;
constexpr uint8_t kVendorOutEp = 0x01U;
constexpr uint8_t kVendorInterface = 0x00U;
constexpr uint16_t kVendorPacketSize = 64U;
constexpr uint8_t kVendorIntervalFs = 0x01U; /* 1 frame = 1 ms */
#if defined(CONFIG_WBR_CONTROL_USB_MPC) && CONFIG_WBR_CONTROL_USB_MPC
constexpr uint8_t kVendorIntervalHs = CONFIG_WBR_CONTROL_USB_MPC_HS_INTERVAL;
#else
constexpr uint8_t kVendorIntervalHs = 0x04U; /* 8 microframes = 1 ms */
#endif

#define WBR_CONTROL_VENDOR_INTERRUPT_DESCRIPTOR_LEN (9 + 7 + 7)
#define WBR_CONTROL_USB_CONFIG_SIZE (9 + WBR_CONTROL_VENDOR_INTERRUPT_DESCRIPTOR_LEN)

static_assert(protocols::kPcCommRecvPacketSize <= kVendorPacketSize,
	      "PC command must fit one Vendor Interrupt OUT packet");
static_assert(protocols::kPcCommSendPacketSize <= kVendorPacketSize,
	      "PC telemetry must fit one Vendor Interrupt IN packet");

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX
uint8_t g_vendor_read_buffer[kVendorPacketSize];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_vendor_tx_buffer[kVendorPacketSize];

volatile bool g_started = false;
volatile bool g_configured = false;
volatile bool g_vendor_tx_busy = false;
volatile bool g_vendor_out_armed = false;

#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
K_SEM_DEFINE(g_pc_link_tx_kick, 0, 1);
volatile bool g_pc_command_clear_requested = true;
bool g_has_valid_pc_command = false;
uint32_t g_last_valid_pc_command_ms = 0U;
uint32_t g_pc_command_sequence = 0U;
#endif

#if defined(CONFIG_WBR_CONTROL_USB_MPC) && CONFIG_WBR_CONTROL_USB_MPC
K_SEM_DEFINE(g_mpc_rx_kick, 0, 1);
volatile uint16_t g_mpc_rx_len = 0U;
#endif

static const uint8_t g_device_descriptor[] = {
	USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t g_config_descriptor_hs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(
		WBR_CONTROL_USB_CONFIG_SIZE,
		0x01,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(kVendorInterface, 0x00, 0x02,
		USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorOutEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalHs),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorInEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalHs),
};

static const uint8_t g_config_descriptor_fs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(
		WBR_CONTROL_USB_CONFIG_SIZE,
		0x01,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(kVendorInterface, 0x00, 0x02,
		USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorOutEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalFs),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorInEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalFs),
};

static const uint8_t g_device_quality_descriptor[] = {
	USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01),
};

static const uint8_t g_other_speed_config_descriptor_hs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(
		WBR_CONTROL_USB_CONFIG_SIZE,
		0x01,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(kVendorInterface, 0x00, 0x02,
		USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorOutEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalFs),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorInEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalFs),
};

static const uint8_t g_other_speed_config_descriptor_fs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(
		WBR_CONTROL_USB_CONFIG_SIZE,
		0x01,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(kVendorInterface, 0x00, 0x02,
		USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorOutEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalHs),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorInEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalHs),
};

static const char *g_string_descriptors[] = {
	(const char[]){0x09, 0x04},
	"HPMicro",
	"wbr_control USB Session",
	"0001",
};

static const uint8_t *DeviceDescriptorCallback(uint8_t speed)
{
	ARG_UNUSED(speed);
	return g_device_descriptor;
}

static const uint8_t *ConfigDescriptorCallback(uint8_t speed)
{
	if (speed == USB_SPEED_HIGH) {
		return g_config_descriptor_hs;
	}

	if (speed == USB_SPEED_FULL) {
		return g_config_descriptor_fs;
	}

	return nullptr;
}

static const uint8_t *DeviceQualityDescriptorCallback(uint8_t speed)
{
	ARG_UNUSED(speed);
	return g_device_quality_descriptor;
}

static const uint8_t *OtherSpeedDescriptorCallback(uint8_t speed)
{
	if (speed == USB_SPEED_HIGH) {
		return g_other_speed_config_descriptor_hs;
	}

	if (speed == USB_SPEED_FULL) {
		return g_other_speed_config_descriptor_fs;
	}

	return nullptr;
}

static const char *StringDescriptorCallback(uint8_t speed, uint8_t index)
{
	ARG_UNUSED(speed);

	if (index >= ARRAY_SIZE(g_string_descriptors)) {
		return nullptr;
	}

	return g_string_descriptors[index];
}

const struct usb_descriptor g_usb_descriptor = {
	.device_descriptor_callback = DeviceDescriptorCallback,
	.config_descriptor_callback = ConfigDescriptorCallback,
	.device_quality_descriptor_callback = DeviceQualityDescriptorCallback,
	.other_speed_descriptor_callback = OtherSpeedDescriptorCallback,
	.string_descriptor_callback = StringDescriptorCallback,
};

int ArmVendorOut()
{
	if (!g_configured || g_vendor_out_armed) {
		return 0;
	}

	const int rc = usbd_ep_start_read(
		kBusId, kVendorOutEp, g_vendor_read_buffer, sizeof(g_vendor_read_buffer));
	if (rc < 0) {
		return rc;
	}

	g_vendor_out_armed = true;
	return 0;
}

void UsbEventHandler(uint8_t busid, uint8_t event)
{
	ARG_UNUSED(busid);

	switch (event) {
	case USBD_EVENT_CONFIGURED:
		g_configured = true;
		g_vendor_tx_busy = false;
		g_vendor_out_armed = false;
	#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
		g_pc_command_clear_requested = true;
		k_sem_give(&g_pc_link_tx_kick);
	#endif
	#if defined(CONFIG_WBR_CONTROL_USB_MPC) && CONFIG_WBR_CONTROL_USB_MPC
		g_mpc_rx_len = 0U;
		k_sem_give(&g_mpc_rx_kick);
	#endif
		(void)ArmVendorOut();
		break;
	case USBD_EVENT_RESET:
		g_configured = false;
		g_vendor_tx_busy = false;
		g_vendor_out_armed = false;
	#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
		g_pc_command_clear_requested = true;
	#endif
	#if defined(CONFIG_WBR_CONTROL_USB_MPC) && CONFIG_WBR_CONTROL_USB_MPC
		g_mpc_rx_len = 0U;
	#endif
		break;
	case USBD_EVENT_DISCONNECTED:
		g_configured = false;
		g_vendor_tx_busy = false;
		g_vendor_out_armed = false;
	#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
		g_pc_command_clear_requested = true;
	#endif
	#if defined(CONFIG_WBR_CONTROL_USB_MPC) && CONFIG_WBR_CONTROL_USB_MPC
		g_mpc_rx_len = 0U;
	#endif
		break;
	default:
		break;
	}
}

#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
void EnqueueUsbRx(const uint8_t *data, size_t len)
{
	const size_t copy_len = MIN(len, msg::kUsbRawChunkSize);
	if ((data == nullptr) || (copy_len == 0U)) {
		return;
	}

	msg::UsbRawFrameMessage frame = {};
	frame.len = static_cast<uint16_t>(copy_len);
	memcpy(frame.data, data, copy_len);
	(void)msg::usb_raw_frames.TryPush(frame);
}
#endif

void UsbInterruptOutCallback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	ARG_UNUSED(busid);

	g_vendor_out_armed = false;
	#if defined(CONFIG_WBR_CONTROL_USB_MPC) && CONFIG_WBR_CONTROL_USB_MPC
	g_mpc_rx_len = static_cast<uint16_t>(MIN(static_cast<size_t>(nbytes),
						    sizeof(g_vendor_read_buffer)));
	k_sem_give(&g_mpc_rx_kick);
	#elif defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
	EnqueueUsbRx(g_vendor_read_buffer, MIN(static_cast<size_t>(nbytes),
					       sizeof(g_vendor_read_buffer)));
	#endif

	/*
	 * Do not reuse the HPM qTD from inside its completion callback.  The HPM
	 * port invokes this callback before USBD_IRQHandler() has finished walking
	 * and retiring the completed qTD.  Re-priming the same endpoint here can
	 * therefore be reported as successful while the new qTD never becomes
	 * active.  PcLinkLoop observes g_vendor_out_armed == false and submits the
	 * next receive after the USB ISR has returned.
	 */
	ARG_UNUSED(ep);
}

void UsbInterruptInCallback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	ARG_UNUSED(busid);
	ARG_UNUSED(ep);
	ARG_UNUSED(nbytes);
	g_vendor_tx_busy = false;
	#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
	k_sem_give(&g_pc_link_tx_kick);
	#endif
}

struct usbd_endpoint g_vendor_out_ep = {
	.ep_addr = kVendorOutEp,
	.ep_cb = UsbInterruptOutCallback,
};

struct usbd_endpoint g_vendor_in_ep = {
	.ep_addr = kVendorInEp,
	.ep_cb = UsbInterruptInCallback,
};

static struct usbd_interface g_vendor_intf;

#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
constexpr uint32_t kPcLinkPeriodMs = CONFIG_WBR_CONTROL_PC_LINK_PERIOD_MS;
constexpr uint32_t kPcCommandTimeoutMs = CONFIG_WBR_CONTROL_PC_LINK_COMMAND_TIMEOUT_MS;

K_THREAD_STACK_DEFINE(g_pc_link_stack, 1536);
k_thread g_pc_link_thread;
bool g_pc_link_started = false;
K_THREAD_STACK_DEFINE(g_pc_link_tx_stack, 1536);
k_thread g_pc_link_tx_thread;

void PublishIdlePcCommand()
{
	msg::PcAutoAimCommand command = {};
	command.sequence = ++g_pc_command_sequence;
	msg::latest_pc_auto_aim_command.write(command);
	g_has_valid_pc_command = false;
}

void PublishAutoAimCommand(const protocols::PCRecvAutoAimData *recv)
{
	msg::PcAutoAimCommand command = {};
	command.sequence = ++g_pc_command_sequence;
	command.mode = recv->mode;
	command.yaw_angle = recv->yaw.yaw_ang;
	command.yaw_velocity = recv->yaw.yaw_vel;
	command.yaw_acceleration = recv->yaw.yaw_acc;
	command.pitch_angle = recv->pitch.pitch_ang;
	command.pitch_velocity = recv->pitch.pitch_vel;
	command.pitch_acceleration = recv->pitch.pitch_acc;
	msg::latest_pc_auto_aim_command.write(command);
	g_last_valid_pc_command_ms = k_uptime_get_32();
	g_has_valid_pc_command = true;
}

void EnforcePcCommandSafety()
{
	if (g_pc_command_clear_requested) {
		g_pc_command_clear_requested = false;
		PublishIdlePcCommand();
		return;
	}

	if (g_has_valid_pc_command &&
	    ((k_uptime_get_32() - g_last_valid_pc_command_ms) > kPcCommandTimeoutMs)) {
		PublishIdlePcCommand();
	}
}

void DrainPcRx()
{
	msg::UsbRawFrameMessage chunk = {};
	while (msg::usb_raw_frames.Pop(chunk, K_NO_WAIT) == 0) {
		if (chunk.len != protocols::kPcCommRecvPacketSize) {
			continue;
		}

		if ((chunk.data[0] != 'S') || (chunk.data[1] != 'P')) {
			continue;
		}

		protocols::PCRecvAutoAimData recv = {};
		if (protocols::DecodePcCommRecv(chunk.data, chunk.len, &recv) == 0) {
			PublishAutoAimCommand(&recv);
		}
	}
}

int SendPcLinkFrame()
{
	protocols::PCSendAutoAimData data = {};

	msg::RemoteInputState input = {};
	if (msg::latest_remote_state.read(input)) {
		data.mode = input.auto_aim ? 1U : 0U;
	}

	msg::Hi91ImuSample imu = {};
	if (msg::latest_hi91_imu_sample.read(imu)) {
		memcpy(data.q, imu.quat, sizeof(data.q));
	}

	msg::GimbalState gimbal = {};
	if (msg::latest_gimbal_state.read(gimbal)) {
		data.yaw.yaw_ang = gimbal.yaw_angle;
		data.yaw.yaw_vel = gimbal.yaw_velocity;
		data.pitch.pitch_ang = gimbal.pitch_angle;
		data.pitch.pitch_vel = gimbal.pitch_velocity;
	}

	msg::BoosterState booster = {};
	if (msg::latest_booster_state.read(booster)) {
		data.bullet.bullet_speed = booster.bullet_speed;
		data.bullet.bullet_count = booster.bullet_count;
	}

	uint8_t packet[protocols::kPcCommSendPacketSize];
	size_t packet_len = 0U;
	if (protocols::EncodePcCommSend(&data, packet, sizeof(packet), &packet_len) != 0) {
		return -EINVAL;
	}

	return platform::SendUsb(packet, packet_len);
}

void PcLinkLoop(void *, void *, void *)
{
	wbr_control::scheduling::AbsolutePeriodicSchedule release(
		kPcLinkPeriodMs, modules::scheduling::thread_phase_ms::kPcLink);
	for (;;) {
		(void)release.WaitForNextRelease();
		if (g_configured && !g_vendor_out_armed) {
			(void)ArmVendorOut();
		}
		EnforcePcCommandSafety();
		DrainPcRx();
	}
}

void PcLinkTxLoop(void *, void *, void *)
{
	for (;;) {
		(void)k_sem_take(&g_pc_link_tx_kick, K_FOREVER);
		if (!g_configured) {
			continue;
		}

		const int rc = SendPcLinkFrame();
		if ((rc < 0) && (rc != -EBUSY) && g_configured) {
			/* A start failure has no completion callback to kick the next try. */
			k_sleep(K_MSEC(1));
			k_sem_give(&g_pc_link_tx_kick);
		}
	}
}

int StartPcLinkWorker()
{
	if (g_pc_link_started) {
		return 0;
	}

	const k_tid_t tx_tid = k_thread_create(
		&g_pc_link_tx_thread, g_pc_link_tx_stack,
		K_THREAD_STACK_SIZEOF(g_pc_link_tx_stack), PcLinkTxLoop,
		nullptr, nullptr, nullptr,
		K_PRIO_PREEMPT(modules::scheduling::thread_priority::kPcLinkTx),
		0, K_NO_WAIT);
	if (tx_tid == nullptr) {
		return -EINVAL;
	}
	k_thread_name_set(tx_tid, "pc_link_tx");

	const k_tid_t tid = k_thread_create(
		&g_pc_link_thread, g_pc_link_stack, K_THREAD_STACK_SIZEOF(g_pc_link_stack),
		PcLinkLoop, nullptr, nullptr, nullptr,
		K_PRIO_PREEMPT(modules::scheduling::thread_priority::kPcLink), 0, K_NO_WAIT);
	if (tid == nullptr) {
		return -EINVAL;
	}

	k_thread_name_set(tid, "pc_link");
	g_pc_link_started = true;
	LOG_INF("pc_link worker started period=%u ms",
		static_cast<unsigned int>(kPcLinkPeriodMs));
	return 0;
}
#endif /* CONFIG_WBR_CONTROL_PC_LINK */

#if defined(CONFIG_WBR_CONTROL_USB_MPC) && CONFIG_WBR_CONTROL_USB_MPC
K_THREAD_STACK_DEFINE(g_usb_mpc_stack, 2048);
k_thread g_usb_mpc_thread;
bool g_usb_mpc_started = false;
uint32_t g_mpc_state_sequence = 0U;
uint32_t g_mpc_received_count = 0U;
uint32_t g_mpc_invalid_count = 0U;
uint32_t g_mpc_tx_busy_count = 0U;
uint32_t g_mpc_out_rearm_failure_count = 0U;
uint32_t g_mpc_last_command_sequence = 0U;
uint64_t g_mpc_last_command_receive_us = 0U;
float g_mpc_last_left_force_n = 0.0F;
float g_mpc_last_right_force_n = 0.0F;
bool g_mpc_have_force_command = false;

constexpr float kMpcMinimumForceN = 20.0F;
constexpr float kMpcMaximumForceN = 150.0F;
constexpr float kMpcMaximumForceStepN = 30.0F;
constexpr uint32_t kMpcMaximumValidForUs = 30000U;
constexpr uint32_t kMpcMaximumStateLag = 2U;
constexpr uint64_t kMpcSessionResetGapUs = 100000U;

bool IsNewerMpcSequence(uint32_t value, uint32_t previous)
{
	return value != previous && static_cast<int32_t>(value - previous) > 0;
}

float ClampMpcForce(float value)
{
	return CLAMP(value, kMpcMinimumForceN, kMpcMaximumForceN);
}

float LimitMpcForceStep(float value, float previous)
{
	return CLAMP(value, previous - kMpcMaximumForceStepN,
		     previous + kMpcMaximumForceStepN);
}

protocols::MpcLinkState MakeMpcLinkState()
{
	const protocols::ethercat::ChassisSample sample =
		modules::EthercatChassisPort::Instance().ReadChassisSample();
	protocols::MpcLinkState state{};
	state.sequence = ++g_mpc_state_sequence;
	state.sender_uptime_ms = k_uptime_get_32();
	state.roll_rad = sample.roll_rad;
	state.roll_rate_rad_s = sample.roll_rate_rad_s;
	state.body_height_m = sample.body_height_m;
	state.body_height_rate_m_s = sample.body_height_rate_m_s;
	state.target_body_height_m = sample.target_body_height_m;
	state.left_leg_length_m = sample.left_leg_length_m;
	state.left_leg_rate_m_s = sample.left_leg_rate_m_s;
	state.right_leg_length_m = sample.right_leg_length_m;
	state.right_leg_rate_m_s = sample.right_leg_rate_m_s;
	state.left_support_force_n = sample.left_support_force_n;
	state.right_support_force_n = sample.right_support_force_n;
	if ((sample.status_flags & protocols::ethercat::kStateImuFresh) != 0U) {
		state.valid_flags |= protocols::kMpcStateImuFresh;
	}
	if ((sample.status_flags & protocols::ethercat::kStateSupportForceValid) != 0U) {
		state.valid_flags |= protocols::kMpcStateSupportForceValid;
	}
	if ((sample.status_flags & protocols::ethercat::kStateMotorFeedbackValid) != 0U) {
		state.valid_flags |= protocols::kMpcStateMotorFeedbackValid;
	}
	if ((sample.status_flags & protocols::ethercat::kStateControlEnabled) != 0U) {
		state.valid_flags |= protocols::kMpcStateControlEnabled;
	}
	if ((sample.status_flags & protocols::ethercat::kStateMpcOperatorEnabled) != 0U) {
		state.valid_flags |= protocols::kMpcStateOperatorEnabled;
	}
	if ((sample.status_flags & protocols::ethercat::kStateMpcActiveApplied) != 0U) {
		state.valid_flags |= protocols::kMpcStateActiveApplied;
	}
	state.motor_stale_mask = sample.motor_stale_mask;
	return state;
}

void EnterUsbMpcFallback()
{
	modules::EthercatChassisPort::Instance().EnterLocalFallback();
	g_mpc_have_force_command = false;
}

void HandleUsbMpcCommand(const protocols::MpcLinkCommand &input)
{
	const uint64_t now_us = k_cyc_to_us_floor64(k_cycle_get_64());
	if (g_mpc_last_command_receive_us == 0U ||
	    now_us - g_mpc_last_command_receive_us > kMpcSessionResetGapUs) {
		g_mpc_last_command_sequence = 0U;
		g_mpc_have_force_command = false;
	}
	if (!IsNewerMpcSequence(input.sequence, g_mpc_last_command_sequence)) {
		return;
	}
	g_mpc_last_command_sequence = input.sequence;
	g_mpc_last_command_receive_us = now_us;
	if (input.mode == protocols::MpcLinkCommandMode::kDisabled) {
		EnterUsbMpcFallback();
		return;
	}
	const bool mode_valid = input.mode == protocols::MpcLinkCommandMode::kShadow ||
		input.mode == protocols::MpcLinkCommandMode::kActive;
	const uint8_t required_flags =
		protocols::kMpcSolverConverged | protocols::kMpcSolverInputFinite;
	const bool flags_valid = (input.solver_flags & required_flags) == required_flags;
	const bool values_finite = isfinite(input.left_support_force_n) &&
		isfinite(input.right_support_force_n);
	const uint32_t state_lag = g_mpc_state_sequence - input.source_state_sequence;
	const bool state_valid = input.source_state_sequence != 0U &&
		state_lag <= kMpcMaximumStateLag;
	const uint32_t validity_us = MIN(
		static_cast<uint32_t>(input.valid_for_ms) * 1000U,
		kMpcMaximumValidForUs);
	if (!mode_valid || !flags_valid || !values_finite || !state_valid ||
	    validity_us == 0U) {
		EnterUsbMpcFallback();
		return;
	}

	float left = ClampMpcForce(input.left_support_force_n);
	float right = ClampMpcForce(input.right_support_force_n);
	const protocols::ethercat::ChassisSample sample =
		modules::EthercatChassisPort::Instance().ReadChassisSample();
	const float previous_left = g_mpc_have_force_command
		? g_mpc_last_left_force_n : ClampMpcForce(sample.left_support_force_n);
	const float previous_right = g_mpc_have_force_command
		? g_mpc_last_right_force_n : ClampMpcForce(sample.right_support_force_n);
	left = LimitMpcForceStep(left, previous_left);
	right = LimitMpcForceStep(right, previous_right);
	protocols::ethercat::ChassisMpcCommand command{};
	command.sequence = input.sequence;
	command.source_state_sequence = input.source_state_sequence;
	command.mode = input.mode == protocols::MpcLinkCommandMode::kActive
		? protocols::ethercat::kMpcActive
		: protocols::ethercat::kMpcShadow;
	command.solver_flags = input.solver_flags;
	command.left_support_force_n = left;
	command.right_support_force_n = right;
	command.valid_for_us = validity_us;
	command.solve_time_us = input.solve_time_us;
	command.received_at_us = now_us;
	if (input.mode == protocols::MpcLinkCommandMode::kActive) {
		modules::EthercatChassisPort::Instance().SetOperatorEnabled(true);
	}
	modules::EthercatChassisPort::Instance().HandleMpcCommand(command);
	g_mpc_last_left_force_n = left;
	g_mpc_last_right_force_n = right;
	g_mpc_have_force_command = true;
}

void ProcessMpcFrame(const uint8_t *data, size_t len)
{
	protocols::MpcLinkCommand command{};
	if (protocols::DecodeMpcLinkCommand(data, len, &command) != 0) {
		++g_mpc_invalid_count;
		return;
	}
	++g_mpc_received_count;
	HandleUsbMpcCommand(command);

	uint8_t packet[protocols::kMpcLinkStateFrameSize];
	size_t packet_len = 0U;
	const protocols::MpcLinkState state = MakeMpcLinkState();
	if (protocols::EncodeMpcLinkState(&state, packet, sizeof(packet), &packet_len) != 0) {
		++g_mpc_invalid_count;
		return;
	}
	const int rc = platform::SendUsb(packet, packet_len);
	if (rc == -EBUSY) {
		++g_mpc_tx_busy_count;
	}
	if ((g_mpc_received_count % 10000U) == 0U) {
		LOG_INF("usb_mpc rx=%u invalid=%u tx_busy=%u rearm_fail=%u",
			g_mpc_received_count, g_mpc_invalid_count,
			g_mpc_tx_busy_count, g_mpc_out_rearm_failure_count);
	}
}

void RearmMpcOutWithRetry()
{
	while (g_configured && !g_vendor_out_armed) {
		const int rc = ArmVendorOut();
		if (rc == 0) {
			return;
		}
		++g_mpc_out_rearm_failure_count;
		/* A transient qTD/endpoint busy result must not leave the event-driven
		 * worker permanently disarmed. Sleep so control/CAN work can run, then
		 * retry without depending on another USB completion to wake us.
		 */
		k_sleep(K_MSEC(1));
	}
}

void UsbMpcLoop(void *, void *, void *)
{
	for (;;) {
		(void)k_sem_take(&g_mpc_rx_kick, K_FOREVER);
		const uint16_t rx_len = g_mpc_rx_len;
		if (rx_len != 0U) {
			ProcessMpcFrame(g_vendor_read_buffer, rx_len);
			g_mpc_rx_len = 0U;
		}
		RearmMpcOutWithRetry();
	}
}

int StartUsbMpcWorker()
{
	if (g_usb_mpc_started) {
		return 0;
	}
	const k_tid_t tid = k_thread_create(
		&g_usb_mpc_thread, g_usb_mpc_stack,
		K_THREAD_STACK_SIZEOF(g_usb_mpc_stack), UsbMpcLoop,
		nullptr, nullptr, nullptr,
		K_PRIO_PREEMPT(modules::scheduling::thread_priority::kUsbMpc),
		0, K_NO_WAIT);
	if (tid == nullptr) {
		return -EINVAL;
	}
	k_thread_name_set(tid, "usb_mpc");
	g_usb_mpc_started = true;
	LOG_INF("usb_mpc worker started priority=%d",
		modules::scheduling::thread_priority::kUsbMpc);
	return 0;
}
#endif /* CONFIG_WBR_CONTROL_USB_MPC */

#endif

}  // namespace

namespace platform {

int InitializeUsbSession()
{
#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE
	if (g_started) {
		return 0;
	}

	if (!kHasUsbNode) {
		return -ENODEV;
	}

	usbd_desc_register(kBusId, &g_usb_descriptor);
	usbd_add_interface(kBusId, &g_vendor_intf);
	usbd_add_endpoint(kBusId, &g_vendor_out_ep);
	usbd_add_endpoint(kBusId, &g_vendor_in_ep);

	const int rc = usbd_initialize(kBusId, kUsbRegBase, UsbEventHandler);
	if (rc < 0) {
		return rc;
	}

	g_started = true;

#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
	{
		const int pc_rc = StartPcLinkWorker();
		if (pc_rc != 0) {
			return pc_rc;
		}
	}
#endif

#if defined(CONFIG_WBR_CONTROL_USB_MPC) && CONFIG_WBR_CONTROL_USB_MPC
	{
		const int mpc_rc = StartUsbMpcWorker();
		if (mpc_rc != 0) {
			return mpc_rc;
		}
	}
#endif

	return 0;
#else
	return -ENOTSUP;
#endif
}

bool IsUsbConfigured()
{
#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE
	return g_started && g_configured;
#else
	return false;
#endif
}

int SendUsb(const uint8_t *data, size_t len)
{
#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE
	if ((data == nullptr) || (len == 0U)) {
		return -EINVAL;
	}

	if (!IsUsbConfigured()) {
		return -EAGAIN;
	}

	if (len > sizeof(g_vendor_tx_buffer)) {
		return -EMSGSIZE;
	}

	if (g_vendor_tx_busy) {
		return -EBUSY;
	}

	memcpy(g_vendor_tx_buffer, data, len);
	g_vendor_tx_busy = true;

	const int rc = usbd_ep_start_write(
		kBusId, kVendorInEp, g_vendor_tx_buffer, static_cast<uint32_t>(len));
	if (rc < 0) {
		g_vendor_tx_busy = false;
		return rc;
	}

	return static_cast<int>(len);
#else
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	return -ENOTSUP;
#endif
}

int ReceiveUsb(uint8_t *out, size_t capacity, size_t *out_len, int32_t timeout_ms)
{
	if ((out == nullptr) || (out_len == nullptr) || (capacity == 0U)) {
		return -EINVAL;
	}

	*out_len = 0U;

#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE
	msg::UsbRawFrameMessage chunk = {};
	const k_timeout_t timeout = timeout_ms < 0 ? K_FOREVER :
		(timeout_ms > 0 ? K_MSEC(timeout_ms) : K_NO_WAIT);
	const int rc = msg::usb_raw_frames.Pop(chunk, timeout);
	if (rc != 0) {
		return rc;
	}

	const size_t copy_len = MIN(capacity, static_cast<size_t>(chunk.len));
	memcpy(out, chunk.data, copy_len);
	*out_len = copy_len;
	return (copy_len == static_cast<size_t>(chunk.len)) ? 0 : -EMSGSIZE;
#else
	ARG_UNUSED(timeout_ms);
	return -ENOTSUP;
#endif
}

}  // namespace platform
