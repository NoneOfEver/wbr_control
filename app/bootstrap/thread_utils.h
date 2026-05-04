/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_BOOTSTRAP_THREAD_UTILS_H_
#define RM_TEST_APP_BOOTSTRAP_THREAD_UTILS_H_

#include <zephyr/kernel.h>

namespace rm_test::app::bootstrap {

template <typename T, void (T::*ThreadMain)()>
inline k_tid_t StartMemberThread(struct k_thread *thread,
				 k_thread_stack_t *stack,
				 size_t stack_size,
				 T *self,
				 int prio,
				 const char *name = nullptr,
				 uint32_t options = 0U,
				 k_timeout_t delay = K_NO_WAIT)
{
	auto entry = [](void *p1, void *p2, void *p3) {
		ARG_UNUSED(p2);
		ARG_UNUSED(p3);

		auto *obj = static_cast<T *>(p1);
		(obj->*ThreadMain)();
	};

	k_tid_t tid = k_thread_create(thread,
				      stack,
				      stack_size,
				      entry,
				      self,
				      nullptr,
				      nullptr,
				      prio,
				      options,
				      delay);
	if (name != nullptr) {
		k_thread_name_set(thread, name);
	}

	return tid;
}

}  // namespace rm_test::app::bootstrap

#endif /* RM_TEST_APP_BOOTSTRAP_THREAD_UTILS_H_ */
