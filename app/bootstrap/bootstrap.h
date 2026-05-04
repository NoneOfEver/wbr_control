/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_BOOTSTRAP_BOOTSTRAP_H_
#define RM_TEST_APP_BOOTSTRAP_BOOTSTRAP_H_

#include <app/bootstrap/module_manager.h>

namespace rm_test::app::bootstrap {

class Bootstrap {
public:
	int Run();

private:
	ModuleManager module_manager_;
};

}  // namespace rm_test::app::bootstrap

#endif /* RM_TEST_APP_BOOTSTRAP_BOOTSTRAP_H_ */
