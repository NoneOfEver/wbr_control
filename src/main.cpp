/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include <channels/system_status_channel.h>
#include <modules/arm/arm_module.h>
#include <modules/chassis/chassis_module.h>
#include <modules/gantry/gantry_module.h>
#include <modules/gimbal/gimbal_module.h>
#include <modules/referee/referee_module.h>
#include <modules/remote_input/remote_input_module.h>
#include <modules/sys_state/sys_state_module.h>
#include <platform/board/board_identity.h>

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) && CONFIG_RM_TEST_RUNTIME_INIT_CAN
#include <platform/drivers/communication/can_dispatch.h>
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS) && CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS
#include <platform/storage/filesystem/littlefs_service.h>
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_UART) && CONFIG_RM_TEST_RUNTIME_INIT_UART
#include <platform/drivers/communication/uart_dispatch.h>
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_USB) && CONFIG_RM_TEST_RUNTIME_INIT_USB
#include <platform/drivers/communication/usb_session.h>
#endif

namespace {

modules::sys_state::SysStateModule g_sys_state_module;
modules::remote_input::RemoteInputModule g_remote_input_module;
modules::chassis::ChassisModule g_chassis_module;
modules::arm::ArmModule g_arm_module;
modules::gimbal::GimbalModule g_gimbal_module;
modules::gantry::GantryModule g_gantry_module;
modules::referee::RefereeModule g_referee_module;

}  // namespace

int main(void)
{
	using channels::SystemStatusMessage;

	printk("rm_test started on %s\n", board_identity_name());

	const SystemStatusMessage booting_status = {
		channels::kBooting,
		0U,
	};

	(void)zbus_chan_pub(&rm_test_system_status_chan, &booting_status, K_NO_WAIT);

	int rc = 0;

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_UART) && CONFIG_RM_TEST_RUNTIME_INIT_UART
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_UART)) {
		rc = platform::drivers::communication::uart_dispatch::Initialize();
		if (rc != 0) {
			printk("uart_dispatch init failed: %d\n", rc);
			return rc;
		}
	}
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) && CONFIG_RM_TEST_RUNTIME_INIT_CAN
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_CAN)) {
		rc = platform::drivers::communication::can_dispatch::Initialize();
		if (rc != 0) {
			if (rc == -ENODEV) {
				printk("can_dispatch init skipped: no CAN device\n");
			} else {
				printk("can_dispatch init failed: %d\n", rc);
				return rc;
			}
		}
	}
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_USB) && CONFIG_RM_TEST_RUNTIME_INIT_USB
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_USB)) {
		rc = platform::drivers::communication::usb_session::Initialize();
		if (rc != 0) {
			if (rc == -ENODEV) {
				printk("usb_session init skipped: no USB device\n");
			} else {
				printk("usb_session init failed: %d\n", rc);
				return rc;
			}
		}
	}
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS) && CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS)) {
		rc = platform::storage::filesystem::littlefs_service::Initialize();
		if (rc != 0) {
			printk("littlefs init skipped: %d\n", rc);
		}
	}
#endif

	uint32_t module_count = 0U;

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_SYS_STATE)) {
		rc = g_sys_state_module.Initialize();
		if (rc != 0) {
			printk("module init failed: %s (%d)\n", g_sys_state_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REMOTE_INPUT)) {
		rc = g_remote_input_module.Initialize();
		if (rc != 0) {
			printk("module init failed: %s (%d)\n", g_remote_input_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_CHASSIS)) {
		rc = g_chassis_module.Initialize();
		if (rc != 0) {
			printk("module init failed: %s (%d)\n", g_chassis_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_ARM)) {
		rc = g_arm_module.Initialize();
		if (rc != 0) {
			printk("module init failed: %s (%d)\n", g_arm_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_GIMBAL)) {
		rc = g_gimbal_module.Initialize();
		if (rc != 0) {
			printk("module init failed: %s (%d)\n", g_gimbal_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_GANTRY)) {
		rc = g_gantry_module.Initialize();
		if (rc != 0) {
			printk("module init failed: %s (%d)\n", g_gantry_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REFEREE)) {
		rc = g_referee_module.Initialize();
		if (rc != 0) {
			printk("module init failed: %s (%d)\n", g_referee_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	const SystemStatusMessage initialized_status = {
		channels::kModulesInitialized,
		module_count,
	};

	(void)zbus_chan_pub(&rm_test_system_status_chan, &initialized_status, K_NO_WAIT);

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_SYS_STATE)) {
		rc = g_sys_state_module.Start();
		if (rc != 0) {
			printk("module start failed: %s (%d)\n", g_sys_state_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REMOTE_INPUT)) {
		rc = g_remote_input_module.Start();
		if (rc != 0) {
			printk("module start failed: %s (%d)\n", g_remote_input_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_CHASSIS)) {
		rc = g_chassis_module.Start();
		if (rc != 0) {
			printk("module start failed: %s (%d)\n", g_chassis_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_ARM)) {
		rc = g_arm_module.Start();
		if (rc != 0) {
			printk("module start failed: %s (%d)\n", g_arm_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_GIMBAL)) {
		rc = g_gimbal_module.Start();
		if (rc != 0) {
			printk("module start failed: %s (%d)\n", g_gimbal_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_GANTRY)) {
		rc = g_gantry_module.Start();
		if (rc != 0) {
			printk("module start failed: %s (%d)\n", g_gantry_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REFEREE)) {
		rc = g_referee_module.Start();
		if (rc != 0) {
			printk("module start failed: %s (%d)\n", g_referee_module.Name(), rc);
			return rc;
		}
	}

	const SystemStatusMessage running_status = {
		channels::kRunning,
		module_count,
	};

	(void)zbus_chan_pub(&rm_test_system_status_chan, &running_status, K_NO_WAIT);

	while (true) {
		k_sleep(K_SECONDS(1));
	}
}
