/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_MODULES_REFEREE_REFEREE_MODULE_H_
#define RM_TEST_APP_MODULES_REFEREE_REFEREE_MODULE_H_

#include <zephyr/kernel.h>

#include <app/bootstrap/module.h>
#include <app/channels/referee_state_channel.h>

namespace rm_test::app::modules::referee {

class RefereeModule : public bootstrap::Module {
public:
	const char *Name() const override { return "referee"; }
	int Initialize() override;
	int Start() override;

private:
	void RunLoop();
	void DecodeUartFramesInQueue();

	struct k_thread thread_;
	bool started_ = false;
	uint32_t sequence_ = 0U;
};

}  // namespace rm_test::app::modules::referee

#endif /* RM_TEST_APP_MODULES_REFEREE_REFEREE_MODULE_H_ */
