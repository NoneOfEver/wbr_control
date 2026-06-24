# CherryUSB CDC ACM Perf Test Debug Report

本文记录 `wbr_control/test/cherryusb_cdc_acm_test` 的调试过程，重点是主机脚本
`tools/usb_cdc_perf.py` 与设备端 `src/cdc_acm.c` 之间的 CDC ACM 测试协议如何一步步修到可用。

## 最终状态

当前三类测试均已跑通：

```sh
python3 tools/usb_cdc_perf.py /dev/cu.usbmodem20240517021 latency
python3 tools/usb_cdc_perf.py /dev/cu.usbmodem20240517021 tx
python3 tools/usb_cdc_perf.py /dev/cu.usbmodem20240517021 rx
```

实测结果示例：

```text
latency command_ping count=200
min=0.062 ms avg=0.087 ms p50=0.083 ms max=0.198 ms

device_to_host=35.61 MiB/s bytes=4194304 host_elapsed=0.112 s

RX_DONE 4194304 509
host_to_device=7.86 MiB/s bytes=4194304 host_elapsed=0.509 s
```

注意：当前 `latency` 测的是 `$PING -> PONG` 命令往返时间，不再测二进制 payload echo。二进制 echo 在本平台上遇到 OUT completion 行为问题，后续需要单独深入。

## 问题 1：TX 一开始读到 ECHO_READY

### 现象

运行：

```sh
python3 tools/usb_cdc_perf.py /dev/cu.usbmodem20240517021 tx
```

报错：

```text
RuntimeError: unexpected response: ECHO_READY
```

### 原因

主机脚本在进入 TX 前会先发 `$ECHO` 同步设备状态。设备回复 `ECHO_READY` 后，串口里可能还残留迟到状态行，或者一次 `os.read()` 读到了多段数据。

旧脚本的问题是：

- `read_line()` 读到换行后直接返回；
- 换行后同一次 read 里多出来的数据没有保留；
- 后续等待 `TX_READY` 时可能读到残留的 `ECHO_READY`。

### 解决

主机脚本引入 `_READ_BACKLOG`：

- `read_line()` 只消费一行，换行后的数据放回 backlog；
- `read_exact()` 优先消费 backlog，避免丢 TX 二进制数据；
- `read_status_line()` 支持忽略迟到状态，如 `ECHO_READY`。

后续又将底层串口从 `os/select/termios` 换成 `pyserial`，代码更清晰，跨 macOS/Linux 更可维护。

## 问题 2：TX_READY 超时

### 现象

修完第一个问题后，TX 有时变成：

```text
TimeoutError: timed out waiting for line
```

卡在等待 `TX_READY`。

### 原因

设备端 `send_status_line()` 不检查 IN endpoint 是否忙。如果刚发完 `ECHO_READY`，马上又要发 `TX_READY`，第二个状态可能在 IN busy 时被覆盖或启动失败。

### 解决

设备端增加状态行排队：

- `pending_status_buffer`
- `pending_status_valid`
- `start_pending_status()`

在 `usbd_cdc_acm_bulk_in()` 中，IN 完成后优先发送 pending status，再继续 TX 数据流。

## 问题 3：TX_READY 迟到污染下一轮

### 现象

一次失败后的下一轮可能在同步 `$ECHO` 时读到旧的：

```text
RuntimeError: unexpected response: TX_READY
```

### 原因

上一轮设备其实已经发出了 `TX_READY`，但主机脚本当时没有正确读到。下一轮开始时，这个迟到状态还在串口缓冲里。

### 解决

`sync_to_echo()` 改成恢复式同步：

- 反复发送 `$ECHO`；
- 遇到 `TX_READY/RX_READY/RX_DONE/PONG/ERR` 等旧状态就丢弃；
- 直到收到真正的 `ECHO_READY`。

这样脚本能从上一轮失败状态恢复。

## 问题 4：RX_READY 超时

### 现象

TX 跑通后，RX 曾经卡在等待：

```text
TimeoutError: timed out waiting for line
```

也就是没有收到 `RX_READY`。

### 原因

CDC OUT 可能把多个短命令合并在同一个 OUT 包中，例如：

```text
$ECHO\n$RX 4194304\n
```

旧设备端命令解析只看一包开头，处理了 `$ECHO` 后，后面的 `$RX` 被忽略。

### 解决

设备端命令解析改为按行处理，并进一步演进为跨 OUT callback 累积：

- `command_buffer[64]`
- `command_len`
- `process_command_bytes()`

它可以处理：

- 一包里多条命令；
- 一条命令被拆成多个 OUT callback；
- 当前处于 `RX_SINK/TX_STREAM` 等测试模式时仍能识别命令边界。

## 问题 5：RX_DONE 被 RX_READY 污染

### 现象

RX 曾经输出：

```text
RX_READY 4194304
host_to_device=...
```

这里本应输出 `RX_DONE ...`。

### 原因

主机脚本为了等待 ready 曾经重发 `$RX`，导致设备可能产生重复 `RX_READY`。发送完数据后，脚本直接 `read_line()`，读到的可能是迟到的 `RX_READY`，而不是 `RX_DONE`。

### 解决

主机脚本改为：

- `$RX/$TX` 命令只发一次；
- RX 数据发送完后显式等待 `RX_DONE`；
- 等 `RX_DONE` 时忽略迟到的 `RX_READY/ECHO_READY`。

## 问题 6：latency 二进制 echo 读到状态行

### 现象

latency 最初出现：

```text
TimeoutError: expected 64 bytes, got 12 bytes
```

12 字节正好是：

```text
ECHO_READY\r\n
```

### 原因

进入 echo 模式时，迟到状态行混入了后续二进制 echo 数据读取路径。

### 解决

主机脚本曾增加状态行过滤：

- `drain_status_lines()`

用于跳过前导的 `ECHO_READY/RX_DONE/TX_READY` 等状态行。

后续二进制 echo 路径已移除，保留 `drain_status_lines()` 用于跨模式同步。

## 问题 7：设备端 echo 回显了错误内存

### 现象

latency mismatch 时看到：

```text
actual=80 06 00 03 00 00 04 00 ...
```

这看起来像 USB control setup 包，不是主机发送的 payload。

### 原因

设备端最初同时 arm 两个 OUT buffer，但 callback 只给 `nbytes`，没有告诉代码完成的是哪块 buffer。代码靠 `read_buffer_index` 交替猜，可能猜错，于是从错误内存回显。

### 解决

设备端暂时改成单 OUT buffer：

- 只使用一个 `read_buffer`；
- callback 里只处理这一个 `read_buffer`；
- 处理完再 `usbd_ep_start_read()`。

同时 echo 不再直接把 OUT buffer 交给 IN endpoint，而是先复制到独立 `echo_buffer`，避免 OUT buffer 重新 arm 后被覆盖。

## 问题 8：二进制 latency echo 没有 OUT 回调

### 现象

即使改成 `$LAT 64`，设备能返回：

```text
LATENCY_READY 64
```

但后续二进制 payload 仍然：

```text
TimeoutError: expected 64 bytes, got 0 bytes
```

尝试过：

- payload 64；
- payload 63；
- payload 65；
- latency 模式按 payload size arm read；
- latency 模式按 1 字节 arm read。

都没有稳定得到二进制 echo。

### 判断

命令 OUT 路径正常，因为 `$LAT`、`$ECHO`、`$PING` 都能工作。问题集中在“主机发送非命令二进制 payload 后，设备端 OUT read/回调没有按预期完成”。

这可能和 HPMicro CherryUSB port 的 OUT transfer 完成条件、CDC ACM tty 层写入行为、或者当前 callback/arm read 策略有关。

### 当前处理

为了先得到可用测试，`latency` 改为 `$PING -> PONG` 命令往返延迟：

```text
running latency test: count=200 command=$PING
```

这样使用已经验证可靠的命令路径，能稳定输出主机观察到的 CDC 命令往返延迟。

## 问题 9：RX 速度低

### 现象

最终结果大约：

```text
device_to_host=35.61 MiB/s
host_to_device=7.86 MiB/s
```

TX 明显快于 RX。

### 原因

RX 当前使用单 OUT buffer。设备端必须等 OUT callback 触发、处理数据、再重新 arm read，中间有空窗。TX 则是在 IN callback 中连续启动下一块发送，流水更顺。

另外 macOS CDC ACM host-to-device 方向也可能受到 tty 层 buffering 和调度影响。

### 验证

尝试不同 host chunk 大小后速度差不多，说明瓶颈不主要在主机 chunk 参数，而在设备端 OUT 接收路径。

### 后续优化方向

要提升 RX，重点不是调 Python chunk，而是研究 HPM/CherryUSB OUT transfer 机制：

1. 是否能可靠同时 queue 多个 OUT request；
2. OUT callback 是否有办法知道完成的是哪个 buffer；
3. 是否可以在不猜 buffer 的前提下恢复双缓冲；
4. 是否需要修改/包装 CherryUSB HPM port，让 callback 带上下文；
5. 或者采用 ring buffer + 单 endpoint request 的更早 re-arm 策略。

当前版本选择正确性优先，因此 RX 速度低于 TX 是预期结果。

## 当前协议总结

主机脚本支持：

```text
ping     -> $PING, expect PONG
latency  -> repeated $PING, measure PONG round trip
rx       -> $RX <bytes>, device sink, expect RX_READY then RX_DONE
tx       -> $TX <bytes>, device stream, expect TX_READY then binary data
```

设备端关键状态：

```text
PERF_MODE_ECHO
PERF_MODE_RX_SINK
PERF_MODE_TX_STREAM
```

`latency` 现在只走 `$PING -> PONG` 命令路径，设备端已经移除二进制
`$LAT` echo 调试模式。

## CherryUSB/HPM 双 OUT queue 调研结论

结论：当前 CherryUSB HPM port 不支持可靠的同 endpoint 双 OUT queue。

本地代码依据：

1. `wbr_control_ws/modules/lib/CherryUSB/port/hpmicro/usb_dc_hpm.c`
   中每个 OUT endpoint 只保存一个 `xfer_buf/xfer_len/actual_xfer_len`。
2. `usbd_ep_start_read()` 每次调用都会覆盖当前 endpoint 的 `xfer_buf`
   和 `xfer_len`，没有 request queue。
3. OUT 完成中断里 cache invalidate 使用当前保存的 `xfer_buf`，如果双提交
   导致指针被覆盖，就可能处理错误 buffer。
4. CherryUSB core 的 endpoint callback 类型是
   `(busid, ep, nbytes)`，不包含完成 buffer 指针或 request context。
5. HPM SDK `usb_device_edpt_xfer()` 会覆盖 qHD 的 `attached_buffer` 和
   `attached_qtd`。qTD 链用于拆分一个大 transfer，不是应用层多个 OUT
   request 排队。
6. Zephyr HPM UDC wrapper 也在 endpoint busy 时返回 `-EBUSY`，说明 HPM
   这条路径按单 in-flight transfer 建模。

因此当前稳定基线采用单 OUT buffer：OUT callback 完成后再重新
`usbd_ep_start_read()`。RX 速度低于 TX 是这个正确性优先设计下的预期现象。

## 方案 B：RX 双 OUT queue 实验开关

为了后续复现实测，当前工程加入了默认关闭的实验开关：

```c
#define PERF_EXPERIMENTAL_DOUBLE_OUT 0
```

开关位置：`src/cdc_acm.c`。

打开为 `1` 后，只有 `rx` 模式会尝试双 OUT：

1. `$RX <bytes>` 进入 `PERF_MODE_RX_SINK`；
2. 首次进入 RX sink 时连续提交两个 `usbd_ep_start_read()`；
3. 每次 OUT completion 后再补一个 read；
4. RX sink 丢弃数据，不依赖完成 buffer 内容；
5. 命令、echo、latency、tx 仍走单 OUT read 基线。

打开后设备 ready 行会变成：

```text
RX_READY <bytes> DOUBLE_OUT
RX_DONE <bytes> <device_elapsed_ms> submit=<n> complete=<n> err=<n>
```

这个实验只用于观察 HPM CherryUSB port 对双 OUT submission 的行为。由于
当前底层不会返回完成 buffer/context，实验结果可能出现 hang、吞吐虚高/虚低、
数据计数异常或后续命令同步失败。稳定测试仍应使用默认值 `0`。

## 调试经验

1. CDC ACM 是串口抽象，但底层仍是 USB bulk endpoint，状态行和二进制流混用时必须有主机侧 backlog。
2. 设备端 IN endpoint busy 时不能直接写下一条状态，必须排队。
3. OUT callback 只给 `nbytes` 时，不能靠交替索引猜 buffer。
4. 性能测试先保正确性，再优化吞吐。单 OUT buffer 慢，但能避免读错内存。
5. 遇到跨模式残留状态时，主机同步逻辑要能恢复，而不是读到旧状态就退出。

## 建议的下一步

如果继续优化：

1. 保留当前版本作为稳定基线；
2. 单独创建一个 RX 双缓冲实验分支；
3. 从 CherryUSB HPM port 查 OUT transfer complete 的数据来源；
4. 确认能定位完成 buffer 后，再恢复多 OUT buffer；
5. 用 `rx --bytes 4194304` 对比单 buffer 与双 buffer 吞吐。
