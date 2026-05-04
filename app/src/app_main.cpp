/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/app_main.h>
#include <app/bootstrap/bootstrap.h>

namespace rm_test::app {

int Main()
{
	bootstrap::Bootstrap bootstrap;

	return bootstrap.Run();
}

}  // namespace rm_test::app
