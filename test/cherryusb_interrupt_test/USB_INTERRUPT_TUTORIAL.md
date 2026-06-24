# CherryUSB Interrupt 实时控制通道调试教程

本文记录 `cherryusb_interrupt_test` 从设计、编码到调通的完整过程。目标不是只说明怎么跑脚本，而是把这次调试里遇到的关键问题、判断依据和修复方式沉淀下来，后续可以作为 HPM + CherryUSB 做 USB interrupt 实时控制帧通讯的参考。

## 1. 背景和目标

这个测试程序用于验证 USB High Speed interrupt endpoint 是否适合做上下位机实时控制帧通讯。

前面的 CDC ACM 测试已经证明：

- device-to-host TX 吞吐可以达到约 `35 MiB/s`。
- host-to-device RX 稳定在约 `7.8 MiB/s`。
- CDC ACM 对调试和日志很方便，但它本质上是串口抽象，控制实时性和帧边界都不如自定义 vendor endpoint 清晰。

实时控制帧更关心：

- 固定周期。
- 低抖动。
- 明确帧边界。
- 不追求很高总吞吐。
- 最好一帧就是一个控制周期的数据。

因此这个目录实现了一个 vendor-specific USB interface，包含：

- interrupt OUT endpoint: `0x01`
- interrupt IN endpoint: `0x81`
- VID/PID: `0x34b7:0xffff`
- interface: `0`

## 2. 为什么选择 Interrupt Endpoint

USB 常见传输类型有 control、bulk、interrupt、isochronous。

### Control

Control transfer 是所有 USB 设备都必须支持的默认控制通道，适合配置、枚举、少量命令。

优点：

- 系统必备。
- 可靠。
- 适合设备配置和少量控制命令。

缺点：

- 不适合高频实时数据流。
- 主机侧 API 和时序通常不适合做主控制循环。

### Bulk

Bulk transfer 适合大吞吐可靠传输，比如 CDC ACM、U 盘、自定义高速数据通道。

优点：

- 吞吐高。
- 有重试，可靠。
- 对大块数据效率好。

缺点：

- 没有固定调度周期保证。
- 在总线繁忙时延迟抖动可能变大。
- 更适合数据流，不是最直观的固定周期控制帧模型。

### Interrupt

Interrupt transfer 不是传统 MCU 中断的意思，而是 USB 主机按描述符声明的 interval 周期性轮询 endpoint。

优点：

- 帧边界清晰。
- 有固定轮询周期。
- HS 下 `bInterval=1` 对应 125 us microframe。
- 非常适合固定小包、低抖动的控制帧。

缺点：

- 吞吐上限低于 bulk。
- 主机控制器按调度表轮询，不能无限提高帧率。
- macOS/PyUSB 同步调用本身也会带来额外开销。

### Isochronous

Isochronous 适合音视频这类定时流。

优点：

- 适合固定带宽、低延迟连续流。

缺点：

- 不保证重传。
- 数据可靠性不如 interrupt/bulk。
- 对实时控制帧通常不合适。

本测试最终选择 interrupt，是因为实时控制更看重稳定周期和低抖动，而不是极限吞吐。

## 3. USB 描述符设计

固件入口文件：

- `src/main.c`
- `src/int_test.c`

核心描述符在 `src/int_test.c`。

High Speed 配置：

```c
#define INT_TEST_PACKET_HS 1024U
#define INT_TEST_INTERVAL  0x01
```

HS interrupt endpoint 最大包长可以到 `1024` 字节。`bInterval=1` 在 High Speed 下表示每个 microframe 轮询一次，也就是理论周期：

```text
125 us = 0.125 ms = 8000 frames/s
```

Full Speed 配置：

```c
#define INT_TEST_PACKET_FS 64U
```

FS interrupt endpoint 最大包长是 `64` 字节。

当前描述符设计为：

```text
HS: wMaxPacketSize = 1024, bInterval = 1
FS: wMaxPacketSize = 64,   bInterval = 1
```

注意：`--size 64` 是测试工具的应用层帧大小，不等于 USB endpoint 的真实 MPS。设备如果 HS 枚举成功，IN endpoint 的 MPS 仍然是 `1024`。

## 4. 测试帧格式

主机和设备使用同一个固定头部：

```text
offset  size  field
0       4     magic
4       4     seq
8       4     host_us / cmd-compatible field
12      4     device_ms / cmd-compatible field
16      4     payload_len
20      4     crc
24      N     payload
```

magic:

```text
0x49544e54
```

命令定义：

```text
0  ECHO
1  STREAM_IN_START
2  STREAM_IN_STOP
3  STREAM_OUT_START
4  STREAM_OUT_STOP
5  STREAM_OUT_DATA
6  RESET
```

当前协议约定：命令字以 `payload[0]` 为准。

这样做的原因是调试过程中发现，主机早期把 `host_us` 当时间戳使用，同时又把命令放到了 payload 中；设备端如果从 `host_us` 取命令，就会把时间戳误判为命令，导致 stream 模式无法启动。

## 5. 固件侧工作方式

设备侧只实现一个 vendor interface，没有 CDC ACM 串口。

启动后：

1. 注册 device/config/string descriptor。
2. 添加 interface。
3. 添加 interrupt OUT endpoint。
4. 添加 interrupt IN endpoint。
5. USB configured 后调用 `usbd_get_ep_mps()` 获取实际 endpoint MPS。
6. arm OUT endpoint，等待主机写入。

### Echo 模式

主机写一帧 OUT，设备回一帧 IN。

适合测试基本连通性：

```sh
python3 tools/usb_interrupt_perf.py --size 64 ping
python3 tools/usb_interrupt_perf.py --size 64 latency --count 200
```

### Stream-in 模式

主机发送 `STREAM_IN_START` 后，设备连续提交 interrupt IN transfer。

设备端每次 IN complete callback 后，如果 stream-in 仍然启用，就提交下一帧。

这用于观察主机能否按 interrupt interval 稳定收到设备帧。

### Stream-out 模式

主机发送 `STREAM_OUT_START` 后，连续写 `STREAM_OUT_DATA`。

设备端只计数，不回包。主机最后发送 `STREAM_OUT_STOP`，设备回一个 stop response，里面的 `seq` 字段表示设备实际收到的数据帧数量。

这用于观察 host-to-device 方向的持续 OUT 能力。

### Reset 命令

每个 stream 测试前后都会发送 `RESET`，用于清理设备状态。

这一步很重要，因为 interrupt IN 可能还有残留响应。如果不 reset，下一轮测试可能读到上一轮留下的帧。

## 6. 主机工具设计

主机工具：

```text
tools/usb_interrupt_perf.py
```

依赖：

```sh
python3 -m pip install pyusb
brew install libusb
```

运行目录：

```sh
cd /Users/panpoming/Documents/zephyr_projects/wbr_control_ws/wbr_control/test/cherryusb_interrupt_test
```

基本命令：

```sh
python3 tools/usb_interrupt_perf.py --size 64 ping
python3 tools/usb_interrupt_perf.py --size 64 latency --count 200
python3 tools/usb_interrupt_perf.py --size 64 exchange --count 1000
python3 tools/usb_interrupt_perf.py --size 64 stream-in --count 2000
python3 tools/usb_interrupt_perf.py --size 64 stream-out --count 2000
```

HS 大包测试：

```sh
python3 tools/usb_interrupt_perf.py --size 1024 stream-in --count 2000
python3 tools/usb_interrupt_perf.py --size 1024 stream-out --count 2000
```

### `--size` 的含义

`--size` 表示应用层测试帧长度，不一定等于 USB 端点 MPS。

例如 HS 枚举下：

```text
endpoint MPS = 1024
--size 64    = 每个测试帧只构造 64 字节
```

主机读 IN 时必须使用 endpoint 真实 MPS 作为 read buffer，否则可能报 overflow。

脚本现在会自动读取 endpoint descriptor：

```python
endpoint_mps(dev, EP_IN)
```

然后使用：

```text
read_size = max(size, endpoint_mps)
```

所以看到输出：

```text
size=64 read_size=1024
```

是正常的。它表示测试帧是 64B，但读缓冲按 HS endpoint MPS 给足。

## 7. 调试中遇到的问题和修复

### 问题 1：CRC 错误

早期 `ping` 可能报：

```text
RuntimeError: bad crc
```

原因是主机和设备计算 CRC 的覆盖范围不一致，或者设备回包时修改了头部字段但没有重新计算 CRC。

修复方式：

- 统一 CRC 只覆盖 `crc` 字段之前的 20 字节。
- 设备修改 `device_ms`、`payload_len` 后重新计算 CRC。

### 问题 2：stream-in 超时

现象：

```text
usb.core.USBTimeoutError: [Errno 60] Operation timed out
```

原因：

主机把命令字放在 `payload[0]`，设备却从 `host_us` 字段解析命令。`host_us` 是时间戳，不是命令，因此设备没有进入 stream-in 模式。

修复方式：

设备端改为：

```c
cmd = (enum int_test_cmd)frame->payload[0];
```

并约定命令字段以 `payload[0]` 为准。

### 问题 3：stream-out stop response 读到旧帧

现象：

```text
cmd mismatch: expected=4 payload_cmd=3
```

原因：

主机在读 stop response 时可能读到之前残留的 start/data/echo 帧。

修复方式：

- 每个测试前后发送 `RESET`。
- reset 前后 drain IN。
- 读取 stop response 时忽略非 stop 帧，直到读到 `STREAM_OUT_STOP` 或超时。

### 问题 4：`Errno 84 Overflow`

现象：

```text
usb.core.USBError: [Errno 84] Overflow
```

原因：

设备 HS interrupt IN endpoint 的 MPS 是 `1024`，但主机使用 `--size 64` 时也用 64 字节 buffer 去读。如果设备实际发出 1024 字节包，libusb 会认为主机 buffer 太小，直接报 overflow。

修复方式：

- `--size` 只控制应用层帧长。
- `dev.read()` 使用 endpoint 真实 MPS 作为 buffer 大小。
- 设备 stream-in 根据 `STREAM_IN_START` 包的实际长度决定后续 IN 帧长度，这样 `--size 64` 真正发送 64B IN 帧。

### 问题 5：同步 ping 延迟看起来不低

早期 `ping/latency` RTT 约 `0.5~0.7 ms`，看起来没有达到 0.125 ms。

原因：

`ping/latency` 是同步 OUT + IN 往返测试，包括：

- Python 调用开销。
- libusb 同步 transfer 开销。
- OUT 调度。
- 设备处理。
- IN 调度。
- 主机线程唤醒。

它测的是往返路径，不是单方向 interrupt endpoint 的总线调度周期。

真正观察 125 us microframe，应使用：

```sh
python3 tools/usb_interrupt_perf.py --size 64 stream-in --count 2000
```

看输出中的 interval。

## 8. 当前测试结果

在 HPM HS USB 上，当前结果：

```text
python3 tools/usb_interrupt_perf.py --size 64 ping
written=64 read=64 seq=0 device_ms=4914 rtt_ms=0.682
```

64B stream-in：

```text
interrupt_stream_in count=2000 size=64 read_size=1024 elapsed=0.294s rate=6811.7 fps in=425.7 KiB/s seq_errors=0
interval: min=0.105 ms avg=0.125 ms p50=0.125 ms p90=0.129 ms p99=0.138 ms max=0.229 ms
```

64B stream-out：

```text
interrupt_stream_out count=2000 size=64 read_size=1024 elapsed=0.250s rate=8000.1 fps out=500.0 KiB/s device_count=2000 cmd=4
```

1024B stream-in：

```text
interrupt_stream_in count=2000 size=1024 read_size=1024 elapsed=0.293s rate=6814.5 fps in=6814.5 KiB/s seq_errors=0
interval: min=0.106 ms avg=0.125 ms p50=0.125 ms p90=0.129 ms p99=0.137 ms max=0.146 ms
```

1024B stream-out：

```text
interrupt_stream_out count=2000 size=1024 read_size=1024 elapsed=0.255s rate=7831.7 fps out=7831.7 KiB/s device_count=2000 cmd=4
```

结论：

- HS interrupt `bInterval=1` 的 125 us 周期已经验证出来。
- `stream-in` 平均间隔 `0.125 ms`，符合 HS microframe。
- `p99` 约 `0.137 ms`，抖动很小。
- `stream-out` 接近 `8000 fps`，达到理论轮询频率附近。
- `seq_errors=0`，说明 IN 连续流没有乱序。
- `device_count=2000`，说明 OUT 连续流完整收到。

## 9. 如何解读几个指标

### `rtt_ms`

`ping` 输出中的 `rtt_ms` 是完整同步往返延迟，不等于单帧 USB interrupt 调度周期。

它适合判断命令响应体验，但不要用它判断 endpoint 的极限周期。

### `interval`

`stream-in` 输出中的 `interval` 是连续 IN 帧到达间隔。

这是判断 interrupt endpoint 调度周期和抖动的关键指标。

如果 HS + `bInterval=1` 工作正常，平均值应该接近：

```text
0.125 ms
```

### `rate`

`rate` 是应用层每秒处理多少帧。

HS interrupt `bInterval=1` 理论上是 `8000 fps`，实际 Python 同步读、libusb、系统调度会让 host 观察到略低的数值。

### `read_size`

`read_size` 是 PyUSB 读缓冲长度，不是应用层帧长度。

HS endpoint MPS 是 1024，所以 `--size 64` 时显示：

```text
size=64 read_size=1024
```

这是正确行为。

## 10. 对实时控制帧的建议

如果控制帧是 64B 或更小，并且希望固定 1 kHz 到 8 kHz 周期，推荐使用当前这种 HS interrupt vendor endpoint。

推荐模型：

- OUT endpoint: 上位机发送控制目标、遥控量、控制模式。
- IN endpoint: 下位机返回状态、时间戳、错误码、传感摘要。
- 每个控制周期固定一个 frame。
- 帧内带 `seq`、时间戳、状态码、CRC。
- 控制主循环不要依赖 CDC ACM。
- CDC ACM 可以保留给日志、调试 shell、低频参数命令。

对于更高吞吐的数据，例如日志流、波形、批量参数、文件传输，仍然建议使用 bulk endpoint 或 CDC ACM。

## 11. 后续可以继续优化的方向

### 1. 异步 PyUSB/libusb 测试

当前脚本使用 PyUSB 同步 API。同步读写方便，但 host 端开销偏大。

如果要进一步压低 host 侧测量误差，可以改为 libusb asynchronous transfer，提前提交多个 IN transfer。

### 2. 设备端双缓冲

当前设备端是 callback 完成后提交下一帧，理论上中间有一点重提交间隙。

如果底层 CherryUSB/HPM interrupt IN 支持可靠排队，可以做双缓冲或预提交。但前面 CDC ACM OUT queue 探索说明 HPM 当前路径并不总是支持真正可靠的多 OUT queue，改之前要确认具体 endpoint 和方向的底层能力。

### 3. 协议结构固化

现在测试协议已经够用，但正式控制协议建议固定成：

```text
magic
version
msg_type
seq
timestamp
payload_len
flags
crc
payload
```

避免继续复用 `host_us/device_ms` 字段承载命令。

### 4. 固件侧时间戳精度

当前设备侧使用 `k_uptime_get_32()`，单位是 ms。正式延迟分析建议改用硬件 cycle counter 或 us 级定时器。

## 12. 推荐的日常验证流程

每次修改固件或脚本后，按下面顺序验证：

```sh
python3 tools/usb_interrupt_perf.py --size 64 ping
python3 tools/usb_interrupt_perf.py --size 64 latency --count 200
python3 tools/usb_interrupt_perf.py --size 64 stream-in --count 2000
python3 tools/usb_interrupt_perf.py --size 64 stream-out --count 2000
python3 tools/usb_interrupt_perf.py --size 1024 stream-in --count 2000
python3 tools/usb_interrupt_perf.py --size 1024 stream-out --count 2000
```

判断标准：

- `ping` 能返回。
- `latency` 无 seq mismatch。
- `stream-in` 的 `seq_errors=0`。
- `stream-in interval avg` 接近 `0.125 ms`。
- `stream-out device_count` 等于 `count`。
- 不出现 `Overflow`、`Timeout`、`cmd mismatch`。

## 13. 当前结论

目前这个测试已经证明：

- HPM 内置 HS USB 可以跑 HS interrupt endpoint。
- `bInterval=1` 下可以观察到稳定的 125 us IN 帧周期。
- 64B 控制帧非常适合使用 interrupt endpoint。
- 1024B interrupt 包也可以跑通，但对实时控制来说通常没有必要每帧都这么大。
- CDC ACM 适合调试和兼容性，实时控制主通道建议使用 vendor-specific interrupt endpoint。

如果后续要做正式上下位机控制协议，可以直接基于这个测试工程继续演进。
