/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_SERVICES_ACTUATOR_ACTUATOR_SERVICE_H_
#define RM_TEST_APP_SERVICES_ACTUATOR_ACTUATOR_SERVICE_H_

#include <stdint.h>

#include <platform/drivers/communication/can_dispatch.h>

namespace rm_test::app::services::actuator {

enum class MotorCurrentGroup : uint8_t {
	kDji0x200 = 0,
	kDji0x1ff,
};

int SendMotorCurrent(rm_test::platform::drivers::communication::can_dispatch::CanBus bus,
		     MotorCurrentGroup group,
		     const int16_t current_cmd[4]);
int SendDjiCurrentGroup200(rm_test::platform::drivers::communication::can_dispatch::CanBus bus,
			   const int16_t current_cmd[4]);
int SendDjiCurrentGroup1ff(rm_test::platform::drivers::communication::can_dispatch::CanBus bus,
			   const int16_t current_cmd[4]);

}  // namespace rm_test::app::services::actuator

#endif /* RM_TEST_APP_SERVICES_ACTUATOR_ACTUATOR_SERVICE_H_ */
