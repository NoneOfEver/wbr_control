# 十状态 MPC：wbr-nuc 纯软件性能实验

2026-10-07 在 wbr-nuc 实测。没有打开 USB、EtherCAT、CAN 或任何执行器接口。
所有远端环境和生成文件位于 `/tmp/wbr-mpc-benchmark`；没有修改生产控制进程。

结论：100 ms 预测时域有可用的计算余量，值得继续做 1 kHz MPC；当前实现尚未满足
每次求解都小于 1 ms。200 ms 时域的当前全消元 OSQP 实现不适合直接作为 1 kHz 控制器。

## 模型和约束

- 复用 `tools/derive_lqr_schedule.py` 的正式十状态、四输入 A/B/Q/R。
- 用当前 CAD 样本插值得到固定 0.25 m 腿长模型；ZOH 离散化。
- 预测步长 5 ms，20/40 个阶段对应 100/200 ms；仿真被控对象每 1 ms 更新。
- 采用离散 DARE 终端代价；消去预测状态。分别为 80/160 个优化变量。
- 每轮力矩 ±16384/3450 Nm；四个关节电机力矩 ±54 Nm。
- 关节约束包含 VMC 支撑力偏置和腿角力矩映射，不用腿角力矩限幅替代实际关节限幅。
  腿部 FK、数值 Jacobian、安装偏移和 VMC 符号来自固件。
- 腿长固定。离线 IK 选择条件较好的代表性分支生成表，在线按测量角度插值。
  它不是实际编码器姿态的重建，实机必须使用真实关节姿态/Jacobian。
- 左右轮合计机械功率使用 `abs(omegaL*tauL)+abs(omegaR*tauR)<=Pmax`。
  用四个符号组合的不等式精确表达这个冻结转速下的约束，不需要辅助变量。
  这是保守的功率绝对值合计，不允许负功率抵消正功率。
- 转速由模型前进速度/yaw rate 做滚动运动学近似，每周期更新、预测时域内冻结。
  未计入精确轮/腿角速度关系、滑移、未来转速变化或电池侧电功率。
- 用户未指定 W 数，40/80 W 都是性能测试参数。
- 每阶段都更新约束矩阵和上下界，包含 OSQP 矩阵更新/重新分解成本。
- OSQP 1.1.3，double，eps_abs=eps_rel=1e-4，最大 400 次迭代，每 10 次检查终止。

## 机器与方法

i7-1185G7，4 核 8 线程；Linux 5.15.0-1114-realtime / PREEMPT_RT。
固定 CPU 2，单线程。使用普通调度，没有设置 SCHED_FIFO、mlockall、隔离核心或锁定频率。
NUC 的无密码 sudo 不可用，未改变机器调度设置。

三个场景：小偏差、较大姿态扰动、高速触发功率限制。每场景 2000 个控制周期，
在第 1000 周期加入姿态/角速度扰动。4 个参数组合，共 24,000 次 Python 软件闭环求解。

原生 C 是同一组闭环 QP 序列的回放，不是原生控制器重新闭环仿真。
每组场景回放前冷启动，保留 warm start。数据在计时前全部读入内存。
每个 QP 原生计时包括上下界、梯度和矩阵更新、重新分解、求解；不包括物理状态处理、
VMC 约束系数构建、结果验证、通信或下位机执行。
Python 计时包含约束系数构建，但不包含仿真状态更新、验证和日志写入。

另做原生 1 ms CLOCK_MONOTONIC 绝对定时回放，检查唤醒延迟和释放后 1 ms 的完成期限。
冷启动及上一周期超期会造成后续周期积压，所以 deadline_misses 不等同于 over_1ms。
统计均为有限样本观测，不是硬实时 WCET 保证。

## 实测结果

以下原生耗时包含矩阵更新和求解，单位 ms。范围取对应时域的全部功率/场景组合。

| 预测时域 | 变量数 | 连续回放平均 | 连续回放 P99 | 最大观测 | 求解超过 1 ms |
|---|---:|---:|---:|---:|---:|
| 100 ms | 80 | 0.223–0.244 | 0.283–0.473 | 2.145 | 4 / 12,000 |
| 200 ms | 160 | 0.995–1.054 | 1.214–1.642 | 5.950 | 1,651 / 12,000 |

| 预测时域 | 1 ms 定时平均 | 1 ms 定时 P99 | 最大观测 | 求解超过 1 ms | 完成期限错过 |
|---|---:|---:|---:|---:|---:|
| 100 ms | 0.456–0.466 | 0.510–0.579 | 3.432 | 6 / 12,000 | 23 / 12,000 |
| 200 ms | 0.996–1.056 | 1.217–1.623 | 6.111 | 1,634 / 12,000 | 12,000 / 12,000 |

定时回放存在频率/缓存/调度影响；没有对这些影响做独立归因。
所有最终 Python 和原生回放求解状态均为 solved，24,000 个软件闭环周期没有触发角度包络重置。
功率约束在高速场景实际激活。54 Nm 关节力矩约束存在且接受验证，但在这些场景中没有贴边激活；
因此这些数据不能代表关节约束频繁激活时的最坏性能。

OSQP 存在数值容差。Python 首输入功率最大超限约 0.00496 W；原生全时域最大功率约束
违反约 0.00809 W。实际应用应预留约束裕量并独立检查输出。

独立验证：`verify.py` 用保留全部预测状态的 QP 对照消元版本，两个测试点的全控制序列
最大差值分别约 7.68e-8 和 5.81e-6 Nm；同时检查两轮功率的四个半空间等价关系。

原始数据保存在 `results/python.json` 与 `results/native.json`。

## 复现

在隔离的 Linux 测试目录中保留本仓库的 `tools` 和这个实验目录。依赖：Python 3、
numpy、scipy、osqp==1.1.3、gcc、taskset。环境初始化不计入在线耗时。

```sh
python3 -m venv /tmp/wbr-mpc-venv
/tmp/wbr-mpc-venv/bin/pip install numpy scipy osqp==1.1.3
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /tmp/wbr-mpc-venv/bin/python test/unified_mpc_nuc/verify.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 taskset -c 2 \
  /tmp/wbr-mpc-venv/bin/python test/unified_mpc_nuc/benchmark.py \
  --cycles 2000 --horizons 20 40 --watts 40 80 \
  --export /tmp/wbr-mpc-export --output /tmp/wbr-mpc-results.json
bash test/unified_mpc_nuc/run_native.sh /tmp/wbr-mpc-export 2
```

在线实现之前需要：真实转速和关节姿态输入、更多腿长/负载/约束贴边测试，
原生约束构建与软件闭环、求解时间上限和可行回退、实时调度，以及完整通信时延测量。
优先继续优化 100 ms 时域版本；200 ms 版本可考虑部分消元或结构化 QP 求解器。

## 2026-10-08 实时调度重测准备

上次 `/tmp` 数据已清空，已在 NUC 的 `/home/wbr-infantry/wbr_mpc_rt_benchmark`
重新生成同样的 24,000 个 QP，重新编译并完成普通调度对照测试。
数据保存在 `results/rt_retest_20261008/baseline.json`。

`replay.c` 新增 `rt` 模式：读入全部数据并预触碰工作页后启用
`mlockall(MCL_CURRENT|MCL_FUTURE)` 和 FIFO 40，设置失败立即报错，
实际 policy、priority、CPU 和 memory_locked 会记录到 stderr。

`run_rt.sh` 是有界的 NUC 专用测试：固定 CPU 2，先测 FIFO/锁内存，
再测 FIFO/锁内存/临时 performance governor，退出时恢复原 governor。
每个回放有 20 秒外部超时，无机器人通信。编译好的四个程序均已准备好。

SSH 用户的实时优先级上限为 0，sudo 需要密码。本次由用户在 NUC 上运行：

```sh
sudo bash /home/wbr-infantry/wbr_mpc_rt_benchmark/wbr_control/test/unified_mpc_nuc/run_rt.sh
```

结果保存在远端 `/home/wbr-infantry/wbr_mpc_rt_benchmark/rt-results-20261008`，
已读取并保存到本地 `results/rt_retest_20261008/realtime.json`。

### 实时重测结果

已确认全部 8 个实时测试进程均为 `policy=1`（SCHED_FIFO）、priority=40、CPU=2、
memory_locked=1；两个 profile 的 governor 分别为 powersave/performance。
运行后实际读取 CPU 2 governor 为 powersave，与保存的原值一致。

各模式、各时域都是 12,000 次 1 ms 定时回放。平均值为六个同样大小场景的总平均；
P99 范围是六个场景各自 P99 的最小/最大值，不是合并样本的 P99。

| 时域 | 模式 | 平均 ms | 场景 P99 ms | 最大 ms | 求解超过 1 ms | 完成期限错过 |
|---|---|---:|---:|---:|---:|---:|
| 100 ms | 普通调度/powersave | 0.456 | 0.524–0.582 | 3.438 | 6 | 28 |
| 100 ms | FIFO/锁内存/powersave | 0.469 | 0.548–0.586 | 3.412 | 6 | 23 |
| 100 ms | FIFO/锁内存/performance | 0.263 | 0.326–0.497 | 2.130 | 4 | 9 |
| 200 ms | 普通调度/powersave | 1.004 | 1.210–1.632 | 5.970 | 1,342 | 12,000 |
| 200 ms | FIFO/锁内存/powersave | 1.016 | 1.212–1.642 | 29.698 | 1,372 | 10,918 |
| 200 ms | FIFO/锁内存/performance | 1.007 | 1.211–1.635 | 5.963 | 1,327 | 10,599 |

所有重测 QP 均返回 solved。100 ms 时域的 FIFO/performance 模式在小偏差与较大姿态
场景合计 8,000 次回放中没有超过求解时限或错过完成期限；四次求解超时来自两个高速
功率场景，包含冷启动和后续扰动，最大迭代次数仍分别为 290/260。

FIFO/锁内存单独没有明显改善求解耗时；叠加 performance 后 100 ms 时域平均耗时降低
约 42%，完成期限错过从 28 降至 9。但只有一次有限样本比较，不能精确分离频率、缓存、
负载和调度影响，也不能据此声明硬实时达标。200 ms 时域仍不满足 1 kHz。
FIFO/powersave 的 200 ms 场景有约 30 ms 的个别延迟；本次没有采集内核跟踪，未归因。

结论：优先继续 100 ms 时域版本，保留 FIFO/内存锁定/performance 配置；剩余超时需要
对功率贴边和扰动时的求解迭代、矩阵更新进行优化，并加入有界可行回退。
通信、真实状态处理和原生 VMC 约束构建仍不在此原生回放计时内。
