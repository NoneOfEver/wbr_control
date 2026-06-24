/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_PLATFORM_STORAGE_FILESYSTEM_LITTLEFS_SERVICE_H_
#define RM_TEST_PLATFORM_STORAGE_FILESYSTEM_LITTLEFS_SERVICE_H_

namespace platform::storage::filesystem::littlefs_service {

int Initialize();
bool IsReady();
const char *MountPoint();

}  // namespace platform::storage::filesystem::littlefs_service

#endif /* RM_TEST_PLATFORM_STORAGE_FILESYSTEM_LITTLEFS_SERVICE_H_ */
