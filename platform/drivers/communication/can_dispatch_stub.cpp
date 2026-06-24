/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <platform/drivers/communication/can_dispatch.h>

namespace rm_test::platform::drivers::communication::can_dispatch {

#if !defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) || (CONFIG_RM_TEST_RUNTIME_INIT_CAN == 0)

int Initialize()
{
	return -ENOTSUP;
}

#endif

}  // namespace rm_test::platform::drivers::communication::can_dispatch
