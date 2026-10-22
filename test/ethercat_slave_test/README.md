# EtherCAT 从站基础已迁移

现有 HPM5E3Y SSC FreeRun 基础独立维护于：
https://github.com/HNUST-Dust/dust-adapter/tree/main/apps/ethercat_slave

本目录不再构建旧独立应用。新仓库保留相同 ESC/PHY 绑定、计数器 PDO 与 ESI；
尚未实现 EtherCAT-to-CAN/UART/IMU 转发，也不包含底盘 MPC 算法。
SSC 5.13 属于外部许可依赖，按新仓库说明导入本地配置源码后构建。
原配置源码可在本仓库 afc2e2d 的 test/ethercat_slave_test/ssc_generated/src
历史版本中查阅；本机迁移时已导入 dust-adapter/external/ssc（不发布）。

保留的 ssc_application_hooks.cpp.template 属于原底盘 MPC 接口参考，不是适配器
固件入口。其他原底盘 EtherCAT/MPC 核心不在本次迁移中删除。
迁移前快照另存 /tmp/wbr-adapter-pre-migration-20261022.tar.gz（本机临时文件）。
