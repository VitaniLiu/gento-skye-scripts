# Gento Skye scripts

机器人遥操、原始数据录制与处理脚本。

新手入门：[Gento Skye遥操录制使用教程](docs/Gento%20Skye遥操录制使用教程.md)。

项目路径：`/home/descfly/gento-skye_scripts`。开发和维护前阅读 `AGENTS.md`。

数据处理入口：`scripts/processing/main.sh`。原始数据留在机器人，主机通过 SSH 只读访问。

LeRobot v2.1 storage and reading rules: [episode storage](docs/lerobot_episode_storage.md).

Independent interactive cleaning: scripts/processing/clean (shell function clean after sourcing deployment/host/convert-shell.bash). Output: lerobot_clean.

Static motion candidate review: clean --preview TASK (read-only videos and plots). Windows local viewer: research workspace 查看clean预览.cmd. Current filter and protection metadata contract: docs/lerobot_episode_storage.md. Quantile missing-action deletion is superseded.


### 键盘 A 补充腰部复位

运行 scripts/teleop/teleop.sh 后，在显示“已就绪”的底盘遥操终端直接按 a 或 A（无需回车），仅将腰部升降复位到 configs/teleop/waist.json 的固定高度0.49631398916244507m。键盘复位不要求松开 LB 或摇杆回中；重复按不会取消，手柄 A 保持原有逻辑。故障、失联或反馈失效时不强制动作，恢复后需重新按键。Ctrl+C 仍退出程序。

2026-11-06 convert now inspects/converts/commits one episode at a time, resumes from persistent checkpoints, reuses unchanged inspection/delivery records and releases episode memory after each commit. Run convert as before; details: docs/lerobot_episode_storage.md.

## Inference

Inference is self-contained in `src/inference`, `configs/inference`, and `scripts/inference`.
It does not import the teleop, replay, recording, or processing implementation.

Robot settings are in `configs/inference/default.json`. A model name selects
`configs/inference/deployed_MODEL.json`, which contains the server connection and
references a `tasks_*.json` task registry. Install the robot endpoint once:

```bash
scripts/inference/install
```

Validate the local configuration without contacting or moving the robot:

```bash
scripts/inference/inference --model default --task toulaji
```

Run real inference explicitly:

```bash
scripts/inference/inference --model default --task toulaji --steps 32 --execute
```

Startup restores Apex Home and the configured waist height, switches the robot to impedance mode, and releases chassis parking. Each reset and infer request includes the task-specific 78-dimensional mask from the inference configuration. Each infer response must return a mask. The client intersects both masks and does not forward masked server fields to the robot. Press `q` to restore upper body and waist, then return the chassis to the measured start position.

## LeRobot replay EEF frames

`scripts/replay/lerobot_replay.py` interprets action[0:9] and action[9:18] as flange_L and flange_R poses relative to base_link, in metres with rotation-matrix first-two-column encoding. These targets are used directly without tool translation or rotation. action[18:27] remains eyes_link. Existing tool-frame datasets require explicit conversion before use.

Native flange replay metadata: schema78.json policy is preferred; export.json is optional legacy fallback. source_mapping.bindings is optional for skye_native_flange_fk_replay_v1; configs/replay/native_flange.json explicitly defines its verified waist origin. Unknown waist mappings are rejected.
