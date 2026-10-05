# can_test

用于在 Zephyr 上验证 CAN 收发链路是否正常。

测试内容：
- 使用 devicetree alias `canbus` 指向的 CAN 设备。
- 默认设为 `CAN_MODE_NORMAL`，bitrate 采用板级 devicetree 值；可通过 `CAN_TEST_LOOPBACK=ON` 改为内部回环。
- 可选拉起 alias `can-transceiver-enable` 指向的 GPIO，用来把外部 CAN 收发器拉出 standby/shutdown。
- 添加标准帧过滤器（ID=`0x123`）。
- 每 500 ms 发送 1 帧标准 CAN 数据，并打印 TX/RX/state/error counter。

## 构建示例

```bash
cmake -S wbr_control/test/can_test -B wbr_control/test/can_test/build -GNinja -DBOARD=dust-hpm6750
cmake --build wbr_control/test/can_test/build -j8
```

HPM5E3Y CAN0 内部回环诊断（无需外部节点，且不会在 CANH/CANL 发出帧）：

```bash
west build -p always -b dust-hpm5e3y -s wbr_control/test/can_test -d wbr_control/build -- -DCAN_TEST_LOOPBACK=ON
```

内部回环应持续看到 `[TX]` 和 `[RX]`。物理总线测试去掉 `-DCAN_TEST_LOOPBACK=ON`，并连接相同波特率、可 ACK 的另一节点。

一次检查 HPM5E3Y 的 CAN0、CAN1、CAN2、CAN3：

```bash
west build -p always -b dust-hpm5e3y -s wbr_control/test/can_test -d wbr_control/build -- -DCAN_TEST_ALL_BUSES=ON
```

此固件依次在四个接口上以 1 Mbit/s 各发送一帧，并打印每路 RXD、GPIO 输入、电平状态和 MCAN 寄存器。即使没有接入可 ACK 的节点，也能用空闲时的 RXD 电平初步判断它们是否卡在同步阶段。

CAN0/1 的 TXD 到 RXD 路径可用短脉冲诊断，并用 CAN2 作对照：

```bash
west build -p always -b dust-hpm5e3y -s wbr_control/test/can_test -d wbr_control/build -- -DCAN_TEST_PIN_PROBE=ON
```

断开外部 CAN 节点后运行。固件把每路 TXD 暂时切为 GPIO，输出 100 微秒低电平，再恢复原有复用；RTT 中的 `[CANx PIN]` 显示 TXD 和 RXD 的读回值。正常收发器通常在 TXD 低期间将 RXD 拉低，并在释放后恢复高电平。

## 收发器 Enable GPIO

如果收发器有 EN/STB/SHDN 管脚且没有硬件默认使能，需要在 board overlay 中添加一个 GPIO 节点，并把 alias 命名为 `can-transceiver-enable`：

```dts
/ {
	aliases {
		canbus = &can1;
		can-transceiver-enable = &can_xcvr_en;
	};

	can_xcvr_en: can-xcvr-en {
		gpios = <&gpioe 26 GPIO_ACTIVE_HIGH>;
	};
};
```

把 `gpioe 26` 和 `GPIO_ACTIVE_HIGH` 改成实际原理图上的管脚和有效电平。若接的是 STB 管脚且高电平表示待机，通常应写成 `GPIO_ACTIVE_LOW`，这样测试程序“assert enable”时会输出物理低电平。

## 预期日志

- 正常发送：持续打印 `[TX] seq=... id=0x123 dlc=8`。
- 无 ACK/总线未接好：可能打印 `tx completion timeout`，随后 state/error counter 会继续输出。
- 收发器未配置 GPIO：会打印 `no can-transceiver-enable alias; assuming transceiver is hardware-enabled`。
