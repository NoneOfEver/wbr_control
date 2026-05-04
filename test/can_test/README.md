# can_test

用于在 Zephyr 上验证 CAN 收发链路是否正常。

测试内容：
- 自动选择一个可用 CAN 设备（`CAN_1/can1/MCAN1/...`）。
- 设为 `CAN_MODE_LOOPBACK`。
- 添加标准帧过滤器（ID=`0x123`）。
- 发送 1 帧数据并等待本机回环接收。
- 比较 `id/dlc/data`，一致则打印 `[PASS]`。

## 构建示例

```bash
cmake -S test/can_test -B test/can_test/build -GNinja -DBOARD=hpm6e00evk_v2
cmake --build test/can_test/build -j8
```

## 预期日志

- 成功：`[PASS] CAN loopback tx/rx ok ...`
- 失败：`[FAIL] ...`（会包含失败步骤与返回码）
