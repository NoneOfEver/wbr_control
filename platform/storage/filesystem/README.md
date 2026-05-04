# Filesystem

这里放文件系统接入层。

当前决策：

- 先使用外置 XPI flash 的 `storage_partition`
- 文件系统选择 LittleFS
- 主要用途是参数文件和日志文件

这样选的原因：

- 板级 DTS 已经存在 `storage_partition`
- 当前私有板定义里没有现成启用的 SDHC 节点
- LittleFS 更适合 NOR flash 上的小文件和断电恢复场景

如果后面确认需要大容量日志或可插拔介质，再补 SDHC + FatFS 方案。
