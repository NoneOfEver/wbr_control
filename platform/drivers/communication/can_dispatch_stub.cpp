/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/sys/util.h>

#include <platform/drivers/communication/can_dispatch.h>

namespace rm_test::platform::drivers::communication::can_dispatch {

#if !defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) || (CONFIG_RM_TEST_RUNTIME_INIT_CAN == 0)

int Initialize()
{
	return -ENOTSUP;
}

int SendStdData(uint16_t can_id, const uint8_t data[8], uint8_t dlc)
{
	ARG_UNUSED(can_id);
	ARG_UNUSED(data);
	ARG_UNUSED(dlc);
	return -ENOTSUP;
}

int SendStdDataOnBus(CanBus bus, uint16_t can_id, const uint8_t data[8], uint8_t dlc)
{
	ARG_UNUSED(bus);
	ARG_UNUSED(can_id);
	ARG_UNUSED(data);
	ARG_UNUSED(dlc);
	return -ENOTSUP;
}

#endif

}  // namespace rm_test::platform::drivers::communication::can_dispatch
