# 2026-11-10 验证记录

示例：G2 yifu episode_index=80，403 帧、20 Hz、20.15 秒。原始 Parquet SHA256：`ff7c91bbd8ad283cba58b65ba0eb8416bfff367e8a80dbef47be1c1ee882332f`，上传解包后匹配。

- 5 项离线测试通过：旋转列约定及坏旋转拒绝、已知可达 FK 目标、无效帧保持/不可达显式标记/速度约束、禁止依赖父功能模块、结构 mask 和时间戳拒绝测试。
- 32 份独立模型资源副本 SHA256 检查通过。
- 12 组随机姿态：独立参考 URDF FK 与 MuJoCo 末端几何最大差异 7.59e-7 m、2.88e-6 rad。
- action、state 各 403 张 JPG，403 个轨迹样本；两个 MP4 解码后均为 403 帧、20.15 秒。
- 浏览器实测：下一帧、上一帧、手动输入帧号跳转至 200、切换 state、播放/暂停；已观察实际渲染图像。输入框的原生编辑和失焦会提交跳转。
- 当前基线：source/target base_link 恒等映射，固定腰高 0.49631398916244507 m，固定底盘，未标定工具。

| 类型 | 左臂通过 | 右臂通过 | 头部通过 |
| --- | --- | --- | --- |
| action | 299/403 | 403/403 | 0/403 |
| state | 301/403 | 403/403 | 0/403 |

通过阈值同时要求位置误差 ≤5 mm、角度误差 ≤0.03 rad。action 左臂最大位置误差 34.92 mm；右臂 0.26 mm；头部 112.00 mm。未通过可能源于本体差异、未标定坐标/工具、运动约束或局部优化，不能仅凭该求解器断言数学上不可达。

结果：`../tests/cross_embodiment/outputs/yifu080/`；详细测试日志、model-audit.json 在 `../tests/cross_embodiment/`。所有帧保留，不发送真机动作。碰撞、接触、动力学、物体/布料、手部重定向和部署仍未实现。网页使用预渲染固定双视角，可精确逐帧，不提供自由 3D 视角。

环境范围：功能所需依赖和实际运行均通过。父项目 `.venv` 的全局 `pip check` 仍报告已安装 LeRobot 0.4.3 缺少 cmake/diffusers/gymnasium/opencv-python-headless/pynput/pyserial/rerun-sdk/termcolor/torchcodec/wandb；本项目直接用 PyArrow 解码，不导入 LeRobot，未扩展安装这些无关依赖。不能把本次功能验证等同于父项目全部环境健康检查通过。
