# rm_test 分层解耦现状报告（更新）

日期：2026-04-05  
范围：applications/rm_test

## 1. 结论摘要

当前工程的分层解耦已进入“主干完成、细节收口”阶段。

现状主干：
- 启动层：main -> app_main -> Bootstrap
- 编排层：ModuleManager（通过 bootstrap 注册钩子注册应用模块）
- 业务层：remote_input / chassis
- 服务层：actuator / chassis_tuning / runtime_init
- 协议层：app/protocols/motors
- 消息层：zbus channels
- 平台层：platform/drivers + platform/storage

综合评估：
- 架构成熟度：较高
- 分层解耦评分：8.3 / 10

与上一版相比，以下问题已关闭：
- 已关闭：chassis 业务层直连 can_dispatch。
- 已关闭：shell 通过模块全局桥接调参。
- 已关闭：core 直接 include modules 注册头。
- 已关闭：Bootstrap 直连 UART/CAN/LittleFS 初始化实现。

---

## 2. 已完成项（按解耦目标）

### 2.1 bootstrap 与 modules 的边界解耦

已完成：
- bootstrap 层通过抽象钩子 `RegisterApplicationModules()` 注册模块，不再直接依赖 modules registry 头文件。
- modules 层承接钩子实现并按 Kconfig 条件注册模块。

关键落点：
- app/bootstrap/include/rm_test/app/bootstrap/module_registry_hook.h
- app/bootstrap/src/module_manager.cpp
- app/modules/modules_registry.cpp

收益：
- bootstrap 层对具体业务模块“只知接口，不知实现目录”。
- 新增/裁剪模块时，核心编排层改动面更小。

### 2.2 启动编排与平台初始化解耦

已完成：
- Bootstrap 仅调用 runtime 初始化服务。
- UART/CAN/LittleFS 初始化收敛到 `runtime_init_service`。

关键落点：
- app/bootstrap/src/bootstrap.cpp
- app/services/runtime/runtime_init_service.h
- app/services/runtime/runtime_init_service.cpp

收益：
- 启动顺序仍集中可控，具体平台初始化细节从 Bootstrap 剥离。

### 2.3 业务执行路径解耦（chassis -> actuator -> platform）

已完成：
- chassis 模块不再直接发送 CAN 帧。
- 电机下发通过 actuator service 中转。

关键落点：
- app/modules/chassis/chassis_module.cpp
- app/services/actuator/actuator_service.h
- app/services/actuator/actuator_service.cpp

收益：
- 业务层与总线发送 API 解绑，后续可在 service 层替换执行后端。

### 2.4 调参与业务模块解耦（shell -> service -> provider）

已完成：
- shell 命令通过 `chassis_tuning_service` 访问 PID 参数。
- chassis 通过 provider 接口注册调参能力。

关键落点：
- app/debug/shell/chassis_tuning_shell.cpp
- app/services/chassis/chassis_tuning_service.h
- app/services/chassis/chassis_tuning_service.cpp
- app/modules/chassis/chassis_module.cpp

收益：
- shell 不再依赖模块命名空间全局桥接函数。

### 2.5 配置裁剪能力（第一阶段）

已完成：
- 模块注册裁剪：`RM_TEST_MODULE_REMOTE_INPUT` / `RM_TEST_MODULE_CHASSIS`。
- shell 调参裁剪：`RM_TEST_SHELL_CHASSIS_TUNING`。
- runtime 初始化行为裁剪：
  - `RM_TEST_RUNTIME_INIT_UART`
  - `RM_TEST_RUNTIME_INIT_CAN`
  - `RM_TEST_RUNTIME_INIT_LITTLEFS`

关键落点：
- Kconfig
- CMakeLists.txt
- app/modules/modules_registry.cpp
- app/services/runtime/runtime_init_service.cpp

说明：
- runtime 已支持编译期裁剪：相关驱动/存储源按 Kconfig 条件加入构建。

---

## 3. 未完成项（当前剩余工作）

### P0（优先）

1. runtime 初始化从“行为裁剪”升级到“编译期裁剪”
- 状态：已完成（2026-04-05）。
- 实现：CMake 按开关条件加入 UART/CAN/LittleFS 相关源文件；runtime 与 actuator 增加编译期防护，避免关闭特性后的链接断裂。
- 结果：默认配置行为不变；关闭特性时可减少对应编译对象与依赖面。

2. 更新文档基线，消除“文档落后代码”
- 状态：已完成（2026-04-05）。
- 实现：重写 AGENT_HANDOFF，移除“仅骨架/未接线”结论，替换为当前已接线架构、条件构建与剩余工作清单。
- 结果：后续协作以当前实现为基线，不再按历史阶段信息推进。

### P1（其次）

3. actuator service 抽象提升
- 状态：进行中（已完成第三步，2026-04-05）。
- 实现：新增通用接口 `SendMotorCurrent(group, current_cmd)`；服务内部改为分发表分发，当前支持 `kDji0x200` 与 `kDji0x1ff` 两个执行组；兼容包装接口保留。
- 后续：继续扩展到非 DJI 协议族，并逐步收敛服务层内部协议分发。

4. tuning service 生命周期完善
- 状态：已完成（2026-04-05）。
- 实现：在原有 `UnregisterProvider(provider)` 与 `HasProvider()` 基础上，升级为多 provider registry；新增 `RegisterProvider(provider, name, priority)`、`ProviderCount()`、`GetProviderStatus()`；服务按 priority 选择 active provider；shell `chassis pid status` 可观测 active provider 名称/优先级/数量。

### P2（演进）

5. include 边界收敛
- 状态：已完成第二步（2026-04-05）。
- 实现：新增 `app/include/rm_test/app/...` 稳定接口头，并将上层 include 迁移到 `rm_test/app/...` 前缀；CMake 已进一步移除 `app` 直曝 include 目录。
- 后续：继续收敛 `app/algorithms` 目录暴露，逐步改为 `rm_test/app/algorithms/...` 稳定前缀。

6. 测试与回放资产补齐
- 状态：已完成第一步（2026-04-05）。
- 实现：新增最小 smoke 回归脚本 `applications/rm_test/tools/smoke_regression.sh` 与文档 `applications/rm_test/docs/SMOKE_REGRESSION.md`，覆盖启动链路、模块注册链路、调参链路可用性，以及默认/CAN-off 构建矩阵。
- 后续：补充行为一致性回放与实机时序测试资产。

---

## 4. 风险评估（当前）

1. 配置开关与构建开关不一致
- 风险：用户以为关闭功能即可减体积，但实际仅跳过初始化逻辑。

2. 服务层单点状态（provider/global）
- 风险：未来多模块扩展时，可能出现注册顺序依赖或重入边界问题。

3. 文档滞后导致的协作偏差
- 风险：后续开发依据过时文档重复做已完成工作或误改架构边界。

---

## 5. 建议的下一步顺序

1. 继续提升 actuator service 抽象层级（P1-3），扩展更多执行组/协议族。
2. 继续完善 tuning service provider 生命周期（P1-4），如需多 provider 则升级 registry。
3. 最后做 include 边界收敛与测试资产补齐（P2）。

---

## 6. 当前结语

rm_test 已从“可运行的迁移主干”推进到“可维护的分层架构主干”。
接下来重点不再是继续铺目录，而是把剩余边界收口（编译期裁剪、服务抽象、生命周期与测试）做扎实。完成这些后，整体解耦度可稳定到 9/10 区间。
