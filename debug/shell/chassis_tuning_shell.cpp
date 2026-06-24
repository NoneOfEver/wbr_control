/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/fs/fs.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

#include <platform/storage/filesystem/littlefs_service.h>
#include <app/services/chassis/chassis_tuning_service.h>

namespace {

constexpr char kPidConfigPath[] = "/lfs/chassis_pid.cfg";

uint32_t FloatToBits(float value)
{
	uint32_t bits = 0U;
	memcpy(&bits, &value, sizeof(bits));
	return bits;
}

float BitsToFloat(uint32_t bits)
{
	float value = 0.0f;
	memcpy(&value, &bits, sizeof(value));
	return value;
}

int32_t ToMilliFixed(float value)
{
	const float scaled = value * 1000.0f;
	if (scaled >= 0.0f) {
		return static_cast<int32_t>(scaled + 0.5f);
	}

	return static_cast<int32_t>(scaled - 0.5f);
}

void MilliToParts(int32_t milli, char *sign, uint32_t *integer, uint32_t *fraction)
{
	if ((sign == nullptr) || (integer == nullptr) || (fraction == nullptr)) {
		return;
	}

	const uint32_t abs_milli = (milli < 0) ? static_cast<uint32_t>(-milli)
						 : static_cast<uint32_t>(milli);
	*sign = (milli < 0) ? '-' : '+';
	*integer = abs_milli / 1000U;
	*fraction = abs_milli % 1000U;
}

int ParsePidFromFloatText(const char *text, float values[5])
{
	char *cursor = const_cast<char *>(text);
	char *end = nullptr;
	for (int i = 0; i < 5; ++i) {
		values[i] = strtof(cursor, &end);
		if (end == cursor) {
			return -EINVAL;
		}
		cursor = end;
	}

	return 0;
}

int ParsePidFromHexBits(const char *text, float values[5])
{
	unsigned int b0 = 0U;
	unsigned int b1 = 0U;
	unsigned int b2 = 0U;
	unsigned int b3 = 0U;
	unsigned int b4 = 0U;
	if (sscanf(text, "%x %x %x %x %x", &b0, &b1, &b2, &b3, &b4) != 5) {
		return -EINVAL;
	}

	values[0] = BitsToFloat(static_cast<uint32_t>(b0));
	values[1] = BitsToFloat(static_cast<uint32_t>(b1));
	values[2] = BitsToFloat(static_cast<uint32_t>(b2));
	values[3] = BitsToFloat(static_cast<uint32_t>(b3));
	values[4] = BitsToFloat(static_cast<uint32_t>(b4));
	return 0;
}

int ParseFloatArg(const char *arg, float *out)
{
	if ((arg == nullptr) || (out == nullptr)) {
		return -EINVAL;
	}

	char *end = nullptr;
	const float value = strtof(arg, &end);
	if ((end == arg) || (*end != '\0')) {
		return -EINVAL;
	}

	*out = value;
	return 0;
}

int CmdPidGet(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	float kp = 0.0f;
	float ki = 0.0f;
	float kd = 0.0f;
	float i_limit = 0.0f;
	float out_limit = 0.0f;

	const int rc = rm_test::app::services::chassis_tuning::GetSpeedPidTuning(
		&kp, &ki, &kd, &i_limit, &out_limit);
	if (rc != 0) {
		shell_error(shell, "get pid failed: %d", rc);
		return rc;
	}

	const int32_t kp_m = ToMilliFixed(kp);
	const int32_t ki_m = ToMilliFixed(ki);
	const int32_t kd_m = ToMilliFixed(kd);
	const int32_t i_limit_m = ToMilliFixed(i_limit);
	const int32_t out_limit_m = ToMilliFixed(out_limit);

	char kp_sign = '+';
	char ki_sign = '+';
	char kd_sign = '+';
	char i_limit_sign = '+';
	char out_limit_sign = '+';
	uint32_t kp_int = 0U, kp_frac = 0U;
	uint32_t ki_int = 0U, ki_frac = 0U;
	uint32_t kd_int = 0U, kd_frac = 0U;
	uint32_t i_limit_int = 0U, i_limit_frac = 0U;
	uint32_t out_limit_int = 0U, out_limit_frac = 0U;
	MilliToParts(kp_m, &kp_sign, &kp_int, &kp_frac);
	MilliToParts(ki_m, &ki_sign, &ki_int, &ki_frac);
	MilliToParts(kd_m, &kd_sign, &kd_int, &kd_frac);
	MilliToParts(i_limit_m, &i_limit_sign, &i_limit_int, &i_limit_frac);
	MilliToParts(out_limit_m, &out_limit_sign, &out_limit_int, &out_limit_frac);

	shell_print(shell,
		    "kp=0x%08x(%c%u.%03u) ki=0x%08x(%c%u.%03u) kd=0x%08x(%c%u.%03u) "
		    "i_limit=0x%08x(%c%u.%03u) out_limit=0x%08x(%c%u.%03u)",
		    (unsigned int)FloatToBits(kp), kp_sign, (unsigned int)kp_int, (unsigned int)kp_frac,
		    (unsigned int)FloatToBits(ki), ki_sign, (unsigned int)ki_int, (unsigned int)ki_frac,
		    (unsigned int)FloatToBits(kd), kd_sign, (unsigned int)kd_int, (unsigned int)kd_frac,
		    (unsigned int)FloatToBits(i_limit), i_limit_sign, (unsigned int)i_limit_int,
		    (unsigned int)i_limit_frac,
		    (unsigned int)FloatToBits(out_limit), out_limit_sign, (unsigned int)out_limit_int,
		    (unsigned int)out_limit_frac);
	return 0;
}

int CmdPidSet(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 6U) {
		shell_error(shell, "usage: chassis pid set <kp> <ki> <kd> <i_limit> <out_limit>");
		return -EINVAL;
	}

	float kp = 0.0f;
	float ki = 0.0f;
	float kd = 0.0f;
	float i_limit = 0.0f;
	float out_limit = 0.0f;

	if ((ParseFloatArg(argv[1], &kp) != 0) ||
	    (ParseFloatArg(argv[2], &ki) != 0) ||
	    (ParseFloatArg(argv[3], &kd) != 0) ||
	    (ParseFloatArg(argv[4], &i_limit) != 0) ||
	    (ParseFloatArg(argv[5], &out_limit) != 0)) {
		shell_error(shell, "invalid float argument");
		return -EINVAL;
	}

	const int rc = rm_test::app::services::chassis_tuning::SetSpeedPidTuning(
		kp, ki, kd, i_limit, out_limit);
	if (rc != 0) {
		shell_error(shell, "set pid failed: %d", rc);
		return rc;
	}

	shell_print(shell, "pid updated");
	return 0;
}

int CmdPidResetI(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const int rc = rm_test::app::services::chassis_tuning::ResetSpeedPidIntegrator();
	if (rc != 0) {
		shell_error(shell, "reset integrator failed: %d", rc);
		return rc;
	}

	shell_print(shell, "integrator reset");
	return 0;
}

int CmdPidStatus(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const char *active_name = nullptr;
	int active_priority = 0;
	size_t provider_count = 0U;
	const int rc = rm_test::app::services::chassis_tuning::GetProviderStatus(
		&active_name, &active_priority, &provider_count);

	if (rc == -ENOENT) {
		shell_print(shell, "provider=not_ready count=0");
		return 0;
	}

	if (rc != 0) {
		shell_error(shell, "provider status failed: %d", rc);
		return rc;
	}

	shell_print(shell,
		    "provider=ready active=%s priority=%d count=%u",
		    active_name,
		    active_priority,
		    (unsigned int)provider_count);
	return 0;
}

int CmdPidSave(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const int mount_rc = rm_test::platform::storage::filesystem::littlefs_service::Initialize();
	if (mount_rc != 0) {
		shell_error(shell, "littlefs init failed: %d", mount_rc);
		return mount_rc;
	}

	float kp = 0.0f;
	float ki = 0.0f;
	float kd = 0.0f;
	float i_limit = 0.0f;
	float out_limit = 0.0f;
	int rc = rm_test::app::services::chassis_tuning::GetSpeedPidTuning(
		&kp, &ki, &kd, &i_limit, &out_limit);
	if (rc != 0) {
		shell_error(shell, "get pid failed: %d", rc);
		return rc;
	}

	fs_file_t file;
	fs_file_t_init(&file);
	rc = fs_open(&file, kPidConfigPath, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (rc != 0) {
		shell_error(shell, "open %s failed: %d", kPidConfigPath, rc);
		return rc;
	}

	char buf[128];
	const int len = snprintk(buf,
				 sizeof(buf),
				 "%08x %08x %08x %08x %08x\n",
				 (unsigned int)FloatToBits(kp),
				 (unsigned int)FloatToBits(ki),
				 (unsigned int)FloatToBits(kd),
				 (unsigned int)FloatToBits(i_limit),
				 (unsigned int)FloatToBits(out_limit));
	if (len <= 0) {
		(void)fs_close(&file);
		return -EIO;
	}

	const int write_len = fs_write(&file, buf, len);
	rc = fs_sync(&file);
	(void)fs_close(&file);
	if (write_len < 0) {
		shell_error(shell, "write %s failed: %d", kPidConfigPath, write_len);
		return write_len;
	}
	if (write_len != len) {
		shell_error(shell, "write %s truncated: %d/%d", kPidConfigPath, write_len, len);
		return -EIO;
	}
	if (rc < 0) {
		shell_error(shell, "sync %s failed: %d", kPidConfigPath, rc);
		return rc;
	}

	shell_print(shell, "pid saved to %s", kPidConfigPath);
	return 0;
}

int CmdPidLoad(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const int mount_rc = rm_test::platform::storage::filesystem::littlefs_service::Initialize();
	if (mount_rc != 0) {
		shell_error(shell, "littlefs init failed: %d", mount_rc);
		return mount_rc;
	}

	fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, kPidConfigPath, FS_O_READ);
	if (rc != 0) {
		shell_error(shell, "open %s failed: %d", kPidConfigPath, rc);
		return rc;
	}

	char buf[128] = {0};
	const ssize_t n = fs_read(&file, buf, sizeof(buf) - 1);
	(void)fs_close(&file);
	if (n <= 0) {
		shell_error(shell, "read %s failed: %d", kPidConfigPath, (int)n);
		return (n == 0) ? -ENOENT : (int)n;
	}

	float values[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	rc = ParsePidFromHexBits(buf, values);
	if (rc != 0) {
		rc = ParsePidFromFloatText(buf, values);
	}
	if (rc != 0) {
		shell_error(shell, "parse %s failed", kPidConfigPath);
		return rc;
	}

	rc = rm_test::app::services::chassis_tuning::SetSpeedPidTuning(
		values[0], values[1], values[2], values[3], values[4]);
	if (rc != 0) {
		shell_error(shell, "apply pid failed: %d", rc);
		return rc;
	}

	shell_print(shell, "pid loaded from %s", kPidConfigPath);
	return 0;
}

int CmdPidDump(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const int mount_rc = rm_test::platform::storage::filesystem::littlefs_service::Initialize();
	if (mount_rc != 0) {
		shell_error(shell, "littlefs init failed: %d", mount_rc);
		return mount_rc;
	}

	fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, kPidConfigPath, FS_O_READ);
	if (rc != 0) {
		shell_error(shell, "open %s failed: %d", kPidConfigPath, rc);
		return rc;
	}

	char buf[128] = {0};
	const ssize_t n = fs_read(&file, buf, sizeof(buf) - 1);
	(void)fs_close(&file);
	if (n <= 0) {
		shell_error(shell, "read %s failed: %d", kPidConfigPath, (int)n);
		return (n == 0) ? -ENOENT : (int)n;
	}

	buf[n] = '\0';
	shell_print(shell, "raw: %s", buf);

	float values[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	int parse_rc = ParsePidFromHexBits(buf, values);
	const char *format = "hex";
	if (parse_rc != 0) {
		parse_rc = ParsePidFromFloatText(buf, values);
		format = "float";
	}

	if (parse_rc == 0) {
		const int32_t kp_m = ToMilliFixed(values[0]);
		const int32_t ki_m = ToMilliFixed(values[1]);
		const int32_t kd_m = ToMilliFixed(values[2]);
		const int32_t i_limit_m = ToMilliFixed(values[3]);
		const int32_t out_limit_m = ToMilliFixed(values[4]);

		char kp_sign = '+';
		char ki_sign = '+';
		char kd_sign = '+';
		char i_limit_sign = '+';
		char out_limit_sign = '+';
		uint32_t kp_int = 0U, kp_frac = 0U;
		uint32_t ki_int = 0U, ki_frac = 0U;
		uint32_t kd_int = 0U, kd_frac = 0U;
		uint32_t i_limit_int = 0U, i_limit_frac = 0U;
		uint32_t out_limit_int = 0U, out_limit_frac = 0U;
		MilliToParts(kp_m, &kp_sign, &kp_int, &kp_frac);
		MilliToParts(ki_m, &ki_sign, &ki_int, &ki_frac);
		MilliToParts(kd_m, &kd_sign, &kd_int, &kd_frac);
		MilliToParts(i_limit_m, &i_limit_sign, &i_limit_int, &i_limit_frac);
		MilliToParts(out_limit_m, &out_limit_sign, &out_limit_int, &out_limit_frac);

		shell_print(shell,
			    "parsed(%s): kp=%c%u.%03u ki=%c%u.%03u kd=%c%u.%03u "
			    "i_limit=%c%u.%03u out_limit=%c%u.%03u",
			    format,
			    kp_sign, (unsigned int)kp_int, (unsigned int)kp_frac,
			    ki_sign, (unsigned int)ki_int, (unsigned int)ki_frac,
			    kd_sign, (unsigned int)kd_int, (unsigned int)kd_frac,
			    i_limit_sign, (unsigned int)i_limit_int, (unsigned int)i_limit_frac,
			    out_limit_sign, (unsigned int)out_limit_int, (unsigned int)out_limit_frac);
	} else {
		shell_warn(shell, "parsed: failed (%d)", parse_rc);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pid,
	SHELL_CMD(status, NULL, "Show chassis tuning provider status", CmdPidStatus),
	SHELL_CMD(get, NULL, "Get chassis speed pid", CmdPidGet),
	SHELL_CMD(set, NULL, "Set chassis speed pid", CmdPidSet),
	SHELL_CMD(save, NULL, "Save pid to littlefs", CmdPidSave),
	SHELL_CMD(load, NULL, "Load pid from littlefs", CmdPidLoad),
	SHELL_CMD(dump, NULL, "Dump raw pid config file", CmdPidDump),
	SHELL_CMD(reset_i, NULL, "Reset pid integrator", CmdPidResetI),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_chassis,
	SHELL_CMD(pid, &sub_pid, "Chassis pid control", NULL),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(chassis, &sub_chassis, "Chassis tuning commands", NULL);

}  // namespace
