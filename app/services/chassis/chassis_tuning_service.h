/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_SERVICES_CHASSIS_CHASSIS_TUNING_SERVICE_H_
#define RM_TEST_APP_SERVICES_CHASSIS_CHASSIS_TUNING_SERVICE_H_

#include <stddef.h>

namespace rm_test::app::services::chassis_tuning {

class SpeedPidTuningProvider {
public:
	virtual ~SpeedPidTuningProvider() = default;
	virtual int SetSpeedPidTuning(float kp, float ki, float kd, float i_limit, float out_limit) = 0;
	virtual int GetSpeedPidTuning(float *kp, float *ki, float *kd, float *i_limit, float *out_limit) = 0;
	virtual int ResetSpeedPidIntegrator() = 0;
};

int RegisterProvider(SpeedPidTuningProvider *provider);
int RegisterProvider(SpeedPidTuningProvider *provider, const char *name, int priority);
int UnregisterProvider(SpeedPidTuningProvider *provider);
bool HasProvider();
size_t ProviderCount();
int GetProviderStatus(const char **active_name, int *active_priority, size_t *provider_count);
int SetSpeedPidTuning(float kp, float ki, float kd, float i_limit, float out_limit);
int GetSpeedPidTuning(float *kp, float *ki, float *kd, float *i_limit, float *out_limit);
int ResetSpeedPidIntegrator();

}  // namespace rm_test::app::services::chassis_tuning

#endif /* RM_TEST_APP_SERVICES_CHASSIS_CHASSIS_TUNING_SERVICE_H_ */
