/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/sys/util.h>

#include <app/modules/arm/arm_module.h>
#include <app/modules/chassis/chassis_module.h>
#include <app/modules/gantry/gantry_module.h>
#include <app/modules/gimbal/gimbal_module.h>
#include <app/modules/referee/referee_module.h>
#include <app/modules/remote_input/remote_input_module.h>
#include <app/modules/sys_state/sys_state_module.h>
#include <app/bootstrap/module_manager.h>
#include <app/bootstrap/module_registry_hook.h>

namespace {

rm_test::app::modules::remote_input::RemoteInputModule g_remote_input_module;
rm_test::app::modules::chassis::ChassisModule g_chassis_module;
rm_test::app::modules::arm::ArmModule g_arm_module;
rm_test::app::modules::gimbal::GimbalModule g_gimbal_module;
rm_test::app::modules::gantry::GantryModule g_gantry_module;
rm_test::app::modules::referee::RefereeModule g_referee_module;
rm_test::app::modules::sys_state::SysStateModule g_sys_state_module;

}  // namespace

namespace rm_test::app::bootstrap {

int RegisterApplicationModules(ModuleManager *manager)
{
	if (manager == nullptr) {
		return -EINVAL;
	}

	/* Keep status indicator first so board state is visible even if later modules fail. */
	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_SYS_STATE)) {
		const int rc = manager->Register(g_sys_state_module);
		if (rc != 0) {
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REMOTE_INPUT)) {
		const int rc = manager->Register(g_remote_input_module);
		if (rc != 0) {
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_CHASSIS)) {
		const int rc = manager->Register(g_chassis_module);
		if (rc != 0) {
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_ARM)) {
		const int rc = manager->Register(g_arm_module);
		if (rc != 0) {
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_GIMBAL)) {
		const int rc = manager->Register(g_gimbal_module);
		if (rc != 0) {
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_GANTRY)) {
		const int rc = manager->Register(g_gantry_module);
		if (rc != 0) {
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REFEREE)) {
		const int rc = manager->Register(g_referee_module);
		if (rc != 0) {
			return rc;
		}
	}

	return 0;
}

}  // namespace rm_test::app::bootstrap
