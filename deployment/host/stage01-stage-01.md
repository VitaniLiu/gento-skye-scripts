# 阶段一：环境核实与隔离目录

已通过主机 SSH 和主机至机器人 SSH 验证连接；未发送运动、模式切换或使能命令。没有使用或存储密码。

实际 SDK 路径：/home/descfly/人形版本_CT040501_SDK040501_260820/SDK/00040501。
主机与机器人均发现 /opt/ApexTeleop。机器人已有 /home/user/apex_record_sync 和 /home/user/apex_recording_bridge；仅核实存在，未验证或修改。新组件尚未部署。
外置 USB 盘 /dev/sda1 挂载 /mnt/apex-data，序列号 WD-WX12A367D1A2，可用空间约 1.8T。机器人 /home/user 所在盘约有 849G 空闲。
/dev/input/by-id 仅观察到 Logitech 键鼠，未发现 GameSir；尚未确认 Pico 连接。

四项功能目录同级，子功能目录内后续采用 main.py 调用组件。当前目录仅为结构，不是已实现的遥操、录制、转换或回放。
生产原始数据目录尚未选定；禁止默认复用旧录制目录。外接盘生产输出计划为 /mnt/apex-data/gento-skye，尚未创建数据。所有测试与输出在 tests/ 下；代码、配置、运行日志均在本项目。现有采集数据未修改、未删除。

旧 apex_n_sharpa 是灵巧手适配项目，不能直接复用其维度及同步假设。其 README 中备份路径与磁盘内容有差异，以实际源码为准。

下一阶段依赖：核查 Apex 实际源码与运行配置、夹爪接口和量纲、Pico 输入映射、GameSir 设备及轴按钮、底盘停止机制、原始录制 schema 与各时钟域。未核实前不生成运动适配或宣称同步成立。实机验证须先通过无发送测试，并明确急停、失联、反馈超时、时限和速度边界。
