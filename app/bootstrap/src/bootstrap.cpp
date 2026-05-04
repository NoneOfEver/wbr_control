/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <app/bootstrap/bootstrap.h>
#include <app/channels/system_status_channel.h>
#include <platform/board/board_identity.h>
#include <app/services/runtime/runtime_init_service.h>

namespace rm_test::app::bootstrap {

int Bootstrap::Run()
{
	printk("rm_test started on %s\n", board_identity_name());

	const channels::SystemStatusMessage booting_status = {
		channels::kBooting,
		0U,
	};

	int rc = 0;

	(void)zbus_chan_pub(&rm_test_system_status_chan, &booting_status, K_NO_WAIT);

	rc = rm_test::app::services::runtime_init::InitializeInfrastructure();
	if (rc != 0) {
		printk("bootstrap failed: InitializeInfrastructure rc=%d\n", rc);
		return rc;
	}

	rc = module_manager_.Initialize();
	if (rc != 0) {
		printk("bootstrap failed: module_manager.Initialize rc=%d\n", rc);
		return rc;
	}

	const channels::SystemStatusMessage initialized_status = {
		channels::kModulesInitialized,
		static_cast<uint32_t>(module_manager_.ModuleCount()),
	};

	(void)zbus_chan_pub(&rm_test_system_status_chan, &initialized_status, K_NO_WAIT);

	rc = module_manager_.Start();
	if (rc != 0) {
		printk("bootstrap failed: module_manager.Start rc=%d\n", rc);
		return rc;
	}

	const channels::SystemStatusMessage running_status = {
		channels::kRunning,
		static_cast<uint32_t>(module_manager_.ModuleCount()),
	};

	(void)zbus_chan_pub(&rm_test_system_status_chan, &running_status, K_NO_WAIT);

	while (true) {
		k_sleep(K_SECONDS(1));
	}
}

}  // namespace rm_test::app::bootstrap
