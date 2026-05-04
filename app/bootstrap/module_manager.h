/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_BOOTSTRAP_MODULE_MANAGER_H_
#define RM_TEST_APP_BOOTSTRAP_MODULE_MANAGER_H_

#include <stddef.h>

namespace rm_test::app::bootstrap {

class Module;

class ModuleManager {
public:
	int Register(Module &module);
	int Initialize();
	int Start();

	size_t ModuleCount() const;

private:
	static constexpr size_t kMaxModules = 16;

	void RegisterBuiltInModules();

	Module *modules_[kMaxModules] = {};
	size_t module_count_ = 0;
	bool builtins_registered_ = false;
};

}  // namespace rm_test::app::bootstrap

#endif /* RM_TEST_APP_BOOTSTRAP_MODULE_MANAGER_H_ */
