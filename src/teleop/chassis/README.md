> 2026-11-02：项目自有 replay 逻辑、入口、配置及专用测试已按要求删除；下文历史回放描述不再代表可用功能。原始录制数据及系统 Apex 组件保留。

# 天机底盘遥操、复位与 Replay

工作目录：`/home/descfly/gento-skye_scripts/src/teleop/chassis`。底盘端对应部署：`/home/user/tianjidipan`。

## 遥操最终命令

```bash
cd /home/descfly/gento-skye_scripts/src/teleop/chassis
./teleop.sh
```

启动后持续记录，无默认总时长限制。L1 + 十字键才发送运动指令。
松开操控并实际停稳约 0.5 秒，结束一条路线记录，程序继续待命。
停稳并松开 L1/方向键后按 A，执行当前混合原路返回。
A 返回成功且确认停车后继续待命，下一轮从当时的静止位置建立基准。
Ctrl+C 关闭整个遥操会话并请求停车；异常、失联和反馈失效仍会停车并可能结束会话。
可选 --seconds N 仍可显式限时，但日常不需要填写。
单次自动返回仍保留 120 秒、距该轮起点 1 米的边界；返回线速度上限 0.05 m/s。

## 其余入口

```bash
./check.sh                  # 只读连接检查，不发布运动指令
python3 -B scripts/run_tests.py
```

当前绑定为 2026-11-02 14:36:43 的遥操去程（不含 A 返回），迁移没有更换路线。
不要再用根目录 sender.py/replay.py 的旧命令；这些源码已经迁入 src。

## 分层结构

- src/teleop：手柄输入、遥操映射、主机入口。
- src/reset：A 键与当前混合返回。
- src/record：分条记录，包含停车余动。
- src/robot：机器人端发布、反馈和停车。
- src/common：通信及项目路径配置。
- config：配置、回放来源、部署校验和迁移基线。
- scripts：同步、检查测试及离线验证工具。
- data/routes：逐条遥操记录；data/comparisons：对比结果。
- logs/sessions：按日期保存完整会话；logs/transport：SSH/ROS 错误。
- reference：不可随意改写的标定参考与参考副本。
- tests：全部离线测试；docs：设计、操作说明。

项目结构和操作说明见本 README；全部数据文件哈希见 data/catalog/file-index.json。
旧日志、参考 JSON 中的路径是历史采集路径；实际迁移映射见 config/layout-baseline.json。
整个迁移只操作 tianjidipan，自始至终没有修改 real-piper 的 egoav 文件。


## 项目内副本与历史资料（2026-11-02）

本副本是后续开发、导入和部署的唯一主机来源。源码内原有 tests/、logs/、data/、reference/ 保留为导入历史；日志差异以 `.source-snapshot-时间戳` 保存，原副本不覆盖。历史回放配置采用副本内相对路径并保留原 SHA-256 锁定，仅显式 replay 读取；普通 teleop 不读取历史路线。A 键仍用于当前会话原路返回，未改变控制算法或默认操作。

集成入口：`~/gento-skye_scripts/scripts/teleop/chassis.sh`（默认离线状态检查），显式 `teleop` 才进入原独立遥操。`check` 会连接机器人但不发布速度；本次验证未运行此连接检查。原 `./teleop.sh` 独立入口保留。

配置：`~/gento-skye_scripts/configs/teleop/chassis.json`。新日志默认 `logs/teleop/`，新路线默认 `logs/teleop/routes/`，均相对于主项目；可用 CHASSIS_LOGS_DIR / CHASSIS_ROUTES_DIR 或配置调整。不得配置到源目录或本副本历史目录。

新测试及输出统一放主项目 tests/。原 scripts/run_tests.py 改为调用主项目隔离测试入口，历史测试本体保留。复制完整性报告与适配前后校验见主项目 tests/chassis_migration/。

部署脚本默认只校验并列出本地负载，`--apply` 才执行原机器人版本/差异保护流程；远端版本不匹配会拒绝写入。未部署、未启动运动服务。旧 config/layout-baseline.json、data/catalog/ 和首次导入清单是历史证据，不是适配后的校验清单。
