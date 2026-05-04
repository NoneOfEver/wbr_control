/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_PLATFORM_DRIVERS_DEVICES_ACTUATORS_SERIAL_SERVO_H_
#define RM_TEST_PLATFORM_DRIVERS_DEVICES_ACTUATORS_SERIAL_SERVO_H_

#include <stdint.h>

namespace rm_test::platform::drivers::devices::actuators::serial_servo {

int Initialize();
int MoveToAngle(uint8_t id, float degrees, uint16_t time_ms);
int SetSpeed(uint8_t id, int16_t speed);
int Stop(uint8_t id);
int ReadId(uint8_t query_id, uint8_t *out_id, uint32_t timeout_ms);

}  // namespace rm_test::platform::drivers::devices::actuators::serial_servo

#endif /* RM_TEST_PLATFORM_DRIVERS_DEVICES_ACTUATORS_SERIAL_SERVO_H_ */
