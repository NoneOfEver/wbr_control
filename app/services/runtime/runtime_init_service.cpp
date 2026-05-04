/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) && CONFIG_RM_TEST_RUNTIME_INIT_CAN
#include <platform/drivers/communication/can_dispatch.h>
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_USB) && CONFIG_RM_TEST_RUNTIME_INIT_USB
#include <platform/drivers/communication/usb_session.h>
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS) && CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS
#include <platform/storage/filesystem/littlefs_service.h>
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_UART) && CONFIG_RM_TEST_RUNTIME_INIT_UART
#include <platform/drivers/communication/uart_dispatch.h>
#endif

#include <app/services/runtime/runtime_init_service.h>

namespace rm_test::app::services::runtime_init {

int InitializeInfrastructure()
{
	int rc = 0;

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_UART) && CONFIG_RM_TEST_RUNTIME_INIT_UART
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_UART)) {
		rc = rm_test::platform::drivers::communication::uart_dispatch::Initialize();
		if (rc != 0) {
			printk("uart_dispatch init failed: %d\n", rc);
			return rc;
		}
	}
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) && CONFIG_RM_TEST_RUNTIME_INIT_CAN
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_CAN)) {
		rc = rm_test::platform::drivers::communication::can_dispatch::Initialize();
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
		rc = rm_test::platform::drivers::communication::usb_session::Initialize();
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
		rc = rm_test::platform::storage::filesystem::littlefs_service::Initialize();
		if (rc != 0) {
			printk("littlefs init skipped: %d\n", rc);
		}
	}
#endif

	return 0;
}

}  // namespace rm_test::app::services::runtime_init
