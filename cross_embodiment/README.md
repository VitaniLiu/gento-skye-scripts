# 跨本体仿真与推理部署

本项目独立拥有源码、配置和模型，不调用父项目 replay / processing / recording / teleop / inference。当前实现第一部分：G2 egoav 78D → Gento Skye 的离线 MuJoCo 运动学可视化，以及基于目标本体原生关节限位的独立回放轨迹生成。第二部分 deployment/ 仅定义未来部署边界。

## 目录

- simulation/：严格数据解码、MuJoCo IK、离屏渲染、静态逐帧网页。
- replay/：独立 LeRobot 解码、FK/IK、目标关节角/速度截断和可选机器人端；不导入旧 replay 等父项目模块。
- configs/：来源 profile、基坐标变换、固定腰高、IK 容差。
- models/skye/：独立模型和网格副本，provenance.json 记录来源和 SHA256。source.xml 是未修改参考；scene.xml 改写独立网格路径并添加地面/目标标记。reference.urdf 提供速度限制。
- deployment/：后续部署接口说明，不执行动作。
- ../tests/cross_embodiment/：输入、测试、缓存和示例结果；不写正式数据目录。

## 使用

在主机 `/home/descfly/gento-skye_scripts/cross_embodiment` 下：

```sh
PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl ../.venv/bin/python -m simulation \
  ../tests/cross_embodiment/input/yifu-episode080-20261110/data/chunk-000/episode_000080.parquet \
  --output ../tests/cross_embodiment/outputs/yifu080
```

默认分别输出 action 和 state；可用 `--fields action`。`--solve-only` 只解算，之后相同参数加 `--render-only` 渲染；缓存要求输入、配置、代码及模型指纹一致。现有结果不会覆盖。

输出 `index.html` + `review-data.js` + action/state 图片目录是无需网络或 Python 服务的静态网页，可直接打开。支持逐帧、源帧号跳转、拖动、倍速、键盘、每帧误差、完整 78 维数值/有效性和三路原始相机对照；双固定视角，不能自由旋转。将整份输出文件夹复制给查看者，不能只复制 HTML。视频 action.mp4 / state.mp4 便于分享；精确帧检查以网页为准。报告 report.json 和轨迹 trajectory.json 保留机器可读结果。

## 数据语义和范围

当前只接受经验证的 `egoav_78d_20261021_v1` 包及其 provenance/source_schema78.json，不猜测任意 LeRobot schema。403 帧示例中位置单位 m，旋转为矩阵前两列。state/action 同行读取，不做标签时间平移，不删帧、不插值。结构 mask 不是动态质量标志；非有限或坏旋转标成 invalid 并保持上一关节姿态。无需 Skye 特有 frame_quality/export 元数据。

0:27 是左右工具和头部目标；27:32 是无物理含义保留位；54、77 夹爪无效；32:54、55:77 手部有效但目标无对应模型，仅展示数值。绝不能把这些槽位按 Skye 自有数据语义处理。

默认基坐标恒等变换，未标定；固定腰高 0.49631398916244507 m、固定底盘。可修改独立配置中的 source_to_target 刚体变换，但应附标定证据。头部只有两个目标关节，无法任意匹配六自由度目标。IK 使用上一帧解为初值，施加 URDF 关节速度和 MJCF 位置限制。起始帧从 Home 解算，不播放虚构的 Home 到首帧过渡。末端误差超过 5 mm 或 0.03 rad 标记 unmatched；继续展示近似姿态并明确报错，不当作可执行轨迹。

本功能没有调用 mj_step，不验证动力学、碰撞、抓取、布料或物体。不同本体冗余结构、工具定义和基坐标标定可能导致大量不可达帧；可视化和关节限位检查是检查证据，不是部署认证。

## 开发

依赖安装在父项目已有 .venv；缓存和 TMPDIR 指向 ../tests/cross_embodiment/。requirements.txt 记录核心版本；requirements-lock.txt 记录验证环境完整版本。运行 `../.venv/bin/python -B ../tests/cross_embodiment/test_simulation.py` 做语义、可达目标、坏输入、隔离和模型几何回归。模型更新必须更新 provenance 并重跑测试。禁止生产代码导入现有父项目功能或发送 ROS/SSH 指令。

## 单 episode 手部掩码派生

`../.venv/bin/python -B -m simulation.remask SOURCE_PARQUET OUTPUT_DIRECTORY` 仅修改 validity：零起始 32–53、55–76 屏蔽，54、77 启用，全部数值和图像保持不变。输出须为根 tests/cross_embodiment 下的新目录。程序流式重写并逐列验证所有非 mask 字段及原文件校验值，记录新的 schema、来源和 change-report.json。解除夹爪掩码不会恢复源数据中缺失的开合轨迹。

该派生 profile 与原 G2 profile 不同，当前仿真 Episode 读取器仍严格限定原 profile，不能直接把派生包传给默认仿真入口；若需要对新夹爪策略仿真，需独立添加并验证适配器。现有 LeRobot replay 的实际离线兼容检查及后续要求见根 tests/cross_embodiment/outputs/yifu080_hand_masked_gripper_enabled/compatibility-notes.md。

## 头部两自由度适配与数据集级 IK 容差

`simulation.adapt_head` 将源头部相对首帧的旋转投影到 Skye yaw/pitch，按目标 URDF 的角度和速度限制截断，再由目标 FK 生成合法的 9D 头部位姿。它不会使用源头部绝对位置，也不能保留目标头部无法表达的 roll 分量；报告会记录投影误差和实际截断次数。

旧 replay 支持从 `meta/export.json` 的 policy 读取 `ik_position_tolerance_m` 和 `ik_rotation_tolerance_rad`。未配置的数据仍使用 5 mm / 0.03 rad；代码上限为 20 mm / 10°，并将该元数据纳入轨迹缓存指纹。本条 yifu 派生数据使用上限值。旧 replay 的完整离线检查仍在左臂源帧 308 因 21.169 mm 位置误差失败；这只是旧逻辑的结果，不是独立回放的验收条件。

## 固定控制量的 replay 数据适配

`../.venv/bin/python -B -m simulation.prepare_fixed SOURCE_PARQUET OUTPUT_DIRECTORY` 接收前述手部屏蔽派生包，将 action/state 的30、31、54、77设为显式零命令并启用对应结构掩码，生成质量、策略和schema元数据。质量仅描述数值/旋转格式及派生命令可用性，不代替IK。该工具不导入原 replay。已生成样例及具体失败诊断见根 tests/cross_embodiment/outputs/yifu080_fixed_controls/diagnosis-and-plan.md；当前读取通过但整段IK尚不通过，不可当作真机可执行文件。

## 独立跨本体回放

`replay/` 对补齐元数据且完成头部适配的数据独立解码。EEF IK 先在较宽数学区间求解，再按照目标 Skye 自有 URDF 的关节角硬限位和速度限制截断。截断后的 EEF 位置/旋转残差写入报告，不再因误差大小拒绝整段轨迹。底盘和左右夹爪命令固定为 0，灵巧手维度必须保持屏蔽，腰部固定。

本机离线生成报告时不连接机器人：

```sh
PYTHONDONTWRITEBYTECODE=1 ../.venv/bin/python -B -m replay.main \
  ../tests/cross_embodiment/outputs/yifu080_fixed_controls_head_clamped/data/chunk-000/episode_000080.parquet \
  --report ../tests/cross_embodiment/outputs/yifu080_independent_replay_report.json
```

真机入口必须显式增加 `--execute`；机器人端会重新读取其安装的 Gento Skye URDF，并在发布前逐项核对限位。程序先根据实时反馈，以五次平滑关节插值移动到轨迹首帧，默认速度不超过各关节 URDF 上限的 10%，到位稳定后任一关节误差超过 0.08 rad 就拒绝进入数据回放。该过渡不进行环境或自碰撞检测，启动前仍须人工确认路径安全并保持急停可用。详见 `replay/README.md` 和 `replay/limit_evidence.json`。
