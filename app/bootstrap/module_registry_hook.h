/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_BOOTSTRAP_MODULE_REGISTRY_HOOK_H_
#define RM_TEST_APP_BOOTSTRAP_MODULE_REGISTRY_HOOK_H_

namespace rm_test::app::bootstrap {

class ModuleManager;

int RegisterApplicationModules(ModuleManager *manager);

}  // namespace rm_test::app::bootstrap

#endif /* RM_TEST_APP_BOOTSTRAP_MODULE_REGISTRY_HOOK_H_ */
