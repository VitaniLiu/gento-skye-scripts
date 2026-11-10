# 独立跨本体回放

本目录只导入同目录模块和 cross_embodiment 自有模型/配置，不导入或调用父项目 replay、processing、recording、teleop、inference/client。实现可以参考旧逻辑，但解码、FK/IK、限位、机器人端均独立维护。

## 策略

- 双臂输入 EEF 的位置和旋转均表示法兰盘坐标系；IK 直接以 URDF `flange_L` / `flange_R` 为目标，不应用 `left_tool` / `right_tool` 的 5 cm 平移及附加旋转。头部仍使用 `eyes_link`。旧 tool 语义报告及原仿真画面不能视为当前 replay 的预览，需重新生成。

- 输入必须具备 `meta/frame_quality.parquet`、`export.json`、`schema78.json`，灵巧手维度屏蔽，底盘 30/31 和夹爪 54/77 有效且数值恒为 0。
- 将三个 EEF 目标独立解为 Skye 双臂和头部关节。求解区间可宽于目标硬件，之后按 cross_embodiment 自有 URDF 的关节角和逐帧速度限制截断。
- 截断后的 EEF 位置/旋转残差写入报告，不再因残差大小中止。非有限数据、缺失掩码、非单调时间、模型/机器人限位不一致仍会中止。
- 底盘保持驻车，不释放底盘、不发送底盘命令；左右夹爪发布固定 0；腰部固定当前反馈值。
- `--execute` 会先从实时反馈姿态平滑移动到截断轨迹首帧，再开始数据回放。过渡采用五次平滑关节插值，速度不超过各关节机器人 URDF 速度上限的 10%，默认至少 4 秒、最多 30 秒；到位稳定 1 秒后，任一关节误差超过 0.08 rad 就拒绝进入数据回放。
- 真机端在发布前读取其安装的 Gento Skye URDF，逐项对比限位，并再次验证所有关节轨迹。

## 离线检查

从项目根目录运行：

```sh
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m cross_embodiment.replay.main \
  tests/cross_embodiment/outputs/yifu080_fixed_controls_head_clamped/data/chunk-000/episode_000080.parquet \
  --report tests/cross_embodiment/outputs/yifu080_cross_replay_report.json
```

只有明确需要真机回放时才增加 `--execute`。离线模式不建立 SSH/ROS 连接。真机模式的 move-to-start 只验证关节限位、速度和到位误差，不提供环境或自碰撞检测，启动前仍须确认路径安全并保持急停可用。

`limit_evidence.json` 记录机器人自带模型的路径、SHA256 和关节限位证据。模型或固件更新后应重新采集证据并验证；任何不一致都会阻止真机发布。
