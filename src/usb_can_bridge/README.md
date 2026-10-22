# USB 网关已迁移

正式固件现在独立维护于：
https://github.com/HNUST-Dust/dust-adapter/tree/main/apps/usb_gateway

本目录不再提供 Zephyr 应用或复制维护驱动。原控制工程共享的 CAN/USB 代码
继续供原控制器使用，但新网关不再依赖此仓库。

在 dust-adapter 的 West workspace 中构建：

```bash
./dust-adapter/scripts/build.sh usb
```

UART10 功能补丁、固定版本依赖和完整步骤见新仓库 README。保留的
UART10_INVESTIGATION.md 仅是历史证据，不是当前构建说明。
旧固件源码可从此仓库 afc2e2d 及更早版本查阅。迁移没有自动刷写板子。
