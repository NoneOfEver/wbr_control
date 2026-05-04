# Communication Adapters

这里放通信传输适配层，而不是旧工程里那种 HAL 风格的全局回调中心。

职责：

- 对接 Zephyr 设备驱动回调
- 将原始收发事件转换成统一的分发入口
- 必要时把底层帧发布到 `zbus` channel
- 不直接持有业务模块对象
