/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/sys/printk.h>

#include <app/bootstrap/module.h>
#include <app/bootstrap/module_manager.h>
#include <app/bootstrap/module_registry_hook.h>

namespace rm_test::app::bootstrap {

int ModuleManager::Register(Module &module)
{
	if (module_count_ >= kMaxModules) {
		return -ENOMEM;
	}

	modules_[module_count_++] = &module;
	return 0;
}

void ModuleManager::RegisterBuiltInModules()
{
	if (builtins_registered_) {
		return;
	}

	const int rc = RegisterApplicationModules(this);
	if (rc != 0) {
		printk("register built-in modules failed: %d\n", rc);
		return;
	}
	builtins_registered_ = true;
}

int ModuleManager::Initialize()
{
	RegisterBuiltInModules();
	if (!builtins_registered_) {
		return -EIO;
	}

	for (size_t i = 0; i < module_count_; ++i) {
		const int rc = modules_[i]->Initialize();

		if (rc != 0) {
			printk("module init failed: %s (%d)\n", modules_[i]->Name(), rc);
			return rc;
		}
	}

	return 0;
}

int ModuleManager::Start()
{
	for (size_t i = 0; i < module_count_; ++i) {
		const int rc = modules_[i]->Start();

		if (rc != 0) {
			printk("module start failed: %s (%d)\n", modules_[i]->Name(), rc);
			return rc;
		}
	}

	return 0;
}

size_t ModuleManager::ModuleCount() const
{
	return module_count_;
}

}  // namespace rm_test::app::bootstrap
