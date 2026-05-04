/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_APP_BOOTSTRAP_MODULE_H_
#define RM_TEST_APP_BOOTSTRAP_MODULE_H_

namespace rm_test::app::bootstrap {

class Module {
public:
	virtual ~Module() = default;

	virtual const char *Name() const = 0;
	virtual int Initialize() { return 0; }
	virtual int Start() = 0;
};

}  // namespace rm_test::app::bootstrap

#endif /* RM_TEST_APP_BOOTSTRAP_MODULE_H_ */
