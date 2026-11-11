# Gento Skye 工程开发规则（冻结）

生效日期：2026-11-02（Asia/Shanghai）。适用于本项目全部开发与维护。
开始任何修改前必须阅读本文件。用户后续明确要求优先于本文件。

## 1. 冻结目录结构

保持当前目录格式。除非任务确有必要，不得新建、删除、重命名或移动既定功能目录。
除 tests/ 外，新文件或子文件夹只能在当前最次级（叶子）目录内创建，不能直接在已有子目录的父目录内新增文件或同级分类。
尤其禁止在项目根目录、src/、configs/、models/、deployment/、scripts/、logs/ 下新增文件或目录。
本次根目录 AGENTS.md 是为持久保存规则而建立的一次性例外，不构成以后新增根文件的先例。
现有 README.md 等文件可按需更新，不需搬迁。

冻结的功能目录如下：
- docs/
- configs/common/、configs/teleop/、configs/recording/、configs/processing/、configs/replay/
- models/robot/、models/calibration/
- deployment/host/、deployment/robot/
- scripts/teleop/、scripts/recording/、scripts/processing/、scripts/replay/
- src/teleop/、src/recording/、src/processing/、src/replay/
- logs/teleop/、logs/recording/、logs/processing/、logs/replay/
- configs/inference/、scripts/inference/、src/inference/、logs/inference/
- tests/（允许按测试用途自由组织子目录）

若叶子目录新增子目录，后续新增文件须遵循新的叶子层级；既有文件不要求为此搬迁。不要仅为规避本规则增加无意义层级。
不得重新引入 common/、src/common/ 或 src/ 根级共享脚本。组件按职责归入五个功能目录，由对应 main.py 调用。
确有必要改变冻结结构时，先说明具体原因、范围与兼容影响；仅改动任务必需部分并同步更新本文件和文档。不能以个人偏好或方便复用作为必要性依据。

## 2. 测试及临时产物

用户所称 test 统一指当前已存在的 tests/；不另外建立 test/，不重命名 tests/。
所有开发测试脚本、测试代码、合成数据、测试日志、测试报告、测试输出、临时下载、临时补丁、调试文件、临时副本和缓存必须位于项目 tests/ 内。
建议分别使用 tests/outputs/、tests/tmp/、tests/cache/；按需创建。
不得在源码目录、docs/、logs/、主机家目录、系统 /tmp 或外接盘写入本项目主动生成的临时或测试文件。
临时目录API必须显式指定 tests/ 下路径；需要时设置 TMPDIR/TMP/TEMP。运行 Python 使用 PYTHONDONTWRITEBYTECODE=1 或将 PYTHONPYCACHEPREFIX 指向 tests/cache/；测试工具缓存及依赖下载缓存同样重定向到 tests/。
检查脚本可经 stdin 执行，但需要落盘时必须存 tests/。测试运行日志不能写正式 logs/。
本规则约束本项目控制的文件输出；不得为此修改第三方既有服务或系统级日志配置。
已存在的外盘 temporary/ 目录保留，但新任务临时产物遵循最新用户规则，使用主机项目 tests/。若生产流程必须同文件系统原子提交而与此限制冲突，先明确冲突，不静默使用外盘 temporary/。

## 3. 环境与正式输出

保留项目独立 .venv/，不改变其他虚拟环境及系统环境。.venv/ 内由环境工具管理的正式安装文件不作为源码布局调整；安装缓存和临时文件仍放 tests/。
正式运行日志位于 logs/<功能>/。正式原始副本、检查报告、转换结果、LeRobot 数据和回放结果按外接盘协议存储，不因测试规则迁回源码目录。
不覆盖已采集数据，不自动删除机器人原始数据。不在代码、配置、文档、日志或测试文件中存认证密码。

## 4. 修改后检查与报告

检查新增文件位置、意外缓存和临时文件，确认 src/ 根目录只有五个指定子目录。
运行与变更相关的必要测试，所有测试产物放 tests/；不得为纯文档修改无故运行实机测试。
报告变更文件、验证结果、未解决问题及下一阶段依赖。未实际验证不得声称通过。
控制开发保持先无发送验证、后具有明确停止条件的实机验证。现阶段不因目录或规则修改发送任何运动指令。

## 2026-11-02 原始数据访问规则更新

原始数据只保存在机器人本机。主机通过SSH只读访问，不创建外接盘raw_data副本，不下载完整原始文件作为转换前提。既有空raw_data目录不写入。外盘仅保存正式转换结果、检查报告、追溯记录及用户明确允许的必要转换中间产物；开发临时文件与测试产物仍全部放tests/。机器人新根目录为/home/user/gento-skye_raw_data，后续录制配置尚未修改。

## gento_skye_78d_v1 转换存储更新

用户明确允许正式转换所需派生产物放外盘 gento-skye_data/temporary/conversions/。原始MCAP仍不得复制；测试转换及其全部输出只能放tests/。正式数据集通过LeRobot标准读取器验证后发布到lerobot/，追溯报告不是replay必需输入。

## 2026-11-04 LeRobot task storage target
User-approved target: task-level LeRobot v2.1, fixed 11 columns and nullable absent wrist cameras. See docs/lerobot_episode_storage.md. Default base-relative conversion now exports task-level v2.1 through src/processing/task_v21.py. Nullable wrist cameras require the documented loader adapter; do not claim unverified stock-reader compatibility.

## 2026-11-04 separate convert and clean
User-approved: convert retains all decodable aligned frames regardless of dynamic validity and records frame_quality/source_frame_mapping sidecars. clean interactively selects task and episode; deletes only runs of all active action dimensions invalid lasting strictly >1s. No episode splitting; rebuild compressed relative timeline and preserve original mapping. Output sibling lerobot_clean; source lerobot read-only. Previous filtering-at-conversion rules are superseded.

## 2026-11-05 quantile clean and visualization
Supersedes the fixed >1s rule: collect all fully action-invalid run lengths in selected episodes, use nearest-rank one-third quantile at ceil(N/3), remove only runs strictly longer than threshold, keep ties/partial missing/valid zero actions. Interactive HTML preview and explicit execute/cancel choice. Never split episodes; synchronized whole-row removal, retain original mapping and compressed output timeline. Original lerobot remains read-only.

## 2026-11-05 body export locks
User requires base rotation (index 31) locked zero for exports. For shouwu, waist height (index 29) is also locked; only body index 30 (forward velocity) enabled. Zero both state/action and invalidate structural/dynamic masks; preserve raw recordings.

## 2026-11-05 static motion candidate clean
Supersedes quantile/action-missing deletion. Follow static_frame_filter_logic.pdf: independent state/action original adjacent differences at 20Hz; active valid channels only; strict speeds, >0.25s runs, retain 3+3 endpoints, endpoint seam limits. Target hold/interpolation metadata must be complete; otherwise preserve all rows and report uncertainty. Hardware-specific gripper/height channels require no change and base velocities must be zero. Source read-only, whole-row keep_indices, candidate lerobot_clean remains unapproved until human review. Reports contain HTTP-review videos, symlog speed plots, mappings and frame stills.

## 2026-11-06 incremental convert and resume
User-approved: inspect one source episode, convert it, verify and commit before advancing. Persistent records under conversions/checkpoints/<task>/; ready artifacts survive interruption. Skip only when source file list/size/mtime, relevant code/mapping/policy fingerprints and verified output agree. Reuse unchanged inspections independently. New task snapshots hard-link unchanged historical image Parquet files, stream sidecars and use disk-backed exact numeric statistics. Task commit journal under conversions/task_transactions repairs interruptions before/between/after directory renames. Release per-episode rows/decoders/Arrow allocations after each commit; delete only this successful episode's derived temporary work directory. Robot raw data and prior verified data remain preserved. Missing legacy quality/source metadata must never be inferred from zero values or treated as a complete checkpoint.

## 2026-11-06 state-only boundary cleanup
User authorizes prefix/suffix trimming when state shows no significant change, even with missing action or incomplete protection provenance. Interior rules remain unchanged. State must be valid/finite; known protected intervals and source/time boundaries stop trimming. Use existing state speed/seam limits plus edge-only gripper .04 units/s and .02 accumulated change, waist height .005 m/s and .001 m accumulated change; base velocity must remain zero. Keep 3 frames toward task; wholly quiet episodes retain 3 frames. Original boundary frames may be cropped. Candidate output and synchronized mapping remain required.

## 2026-11-08 bounded clean batches
Clean reads/verifies one episode at a time, releases image rows, and durably checkpoints every 10 episodes (including final short batch) under conversions/clean_checkpoints/<task>/<source-selection-code-key>/. Resume only verified matching batches. Final task metadata/statistics/report and publication follow completion of all batches; preserve source and previous clean output. Display processed and saved counts separately. --resume-report reuses an existing confirmed review plan without regenerating media. Numeric task statistics use disk-backed scratch. Tests and backups remain under tests/.

## 2026-11-08 toulaji body channels
User explicitly enables toulaji body indices 29,30,31 (waist vertical, base forward, base rotation). Indices 27,28 remain locked. Other tasks retain their configured policies; base rotation defaults locked_zero and is enabled only by task policy. Preserve existing clockwise output sign convention.

## 2026-11-09 independent inference client
User authorizes a fifth independent inference feature under configs/scripts/src/logs inference directories. It must not import existing teleop, processing, recording, or replay code and must read only inference-owned configuration. Before inference it restores Apex Home with the inference-owned teleop default waist height, switches to impedance mode, and releases chassis parking. The client sends a task-specific 78-dimensional mask to the server on reset and infer. Every infer response must include a mask; the client intersects it with the task mask and must not forward masked server fields to the robot. Pressing q restores upper body and waist first, then returns the chassis to its recorded start. Real motion remains explicit through --execute.

## 2026-11-09 inference client live test exception
The user explicitly requires this experiment under `test/client_test/`, which is a narrow exception to the project-wide `tests/` naming rule. Store protocol files and conversion scratch only under `test/client_test/data/`. The test has three commands: `convert` reads one complete robot-local raw MCAP Episode and writes chunked request/recorded-response protocol JSONL without a server or motion; `replay` passes that unmodified protocol response data through the inference client's production response mask/IK conversion and robot endpoint; `run` performs convert then replay. It must call production inference request construction, response conversion and robot dispatch rather than copying them. Protocol waist height is relative to the recorded 0.4756409823894501 m origin and the inference client converts it to/from the absolute robot joint height. `replay` and `run` require explicit `--execute`, support q, perform the standard startup reset, and perform the standard body/waist then chassis cleanup.


## 2026-11-10 independent cross-embodiment project
User requests an independent project for cross-embodiment simulation and inference deployment. This necessary scope extension authorizes `cross_embodiment/` with simulation/, deployment/, configs/, models/skye/, docs/ and project README/requirements. It does not relocate or change existing functional directories. Production code must not import or invoke replay, processing, recording, teleop or inference modules; independently owned copies and assets must record provenance. deployment/ is documentation-only until explicitly implemented. All example inputs, test code, caches, temporary files, reports and renders remain in root tests/cross_embodiment/. Use the existing root .venv; do not change system environments. Simulation is offline and must never publish robot commands. Preserve all source rows, validity semantics and timestamps; distinguish source and target embodiments, failed IK, unsupported channels and uncalibrated transforms.

## 2026-11-10 independent cross-embodiment replay

User explicitly authorizes `cross_embodiment/replay/` as the independent cross-embodiment replay implementation. It may reuse ideas or copied code with provenance but must not import or invoke parent replay, processing, recording, teleop, inference/client scripts. It owns decoding, kinematics, limit policy and robot endpoint. Gento Skye robot-owned URDF limits are authoritative: solve EEF IK, clamp joint angles and per-frame rates to those limits, report EEF residuals without rejecting for their magnitude. Fixed base and gripper commands remain zero. Real motion always requires explicit `--execute`; before publishing, the robot endpoint must verify its installed URDF limits and every trajectory angle. Offline validation must not establish SSH/ROS control connections. Test artifacts and reports remain under root tests/cross_embodiment/.

For `--execute`, the independent endpoint may move from measured robot joints to the clipped first frame before replay. The transition must use a zero-end-velocity smooth interpolation, derive its speed bound from the installed URDF velocity limits, preserve fixed-zero grippers/base and fixed waist, verify measured arrival before replay, and stop cleanly on stdin EOF. This transition does not claim collision checking; documentation must keep that limitation explicit.


## 2026-11-11 development branch

User designates `develop` in `https://github.com/VitaniLiu/gento-skye-scripts` as the code development branch. Commit and push subsequent code changes to `develop` first; do not push directly to `main`. The live source is `/home/descfly/gento-skye_scripts`; the Git checkout used for publication is `tests/github-publish-20261111`. Synchronize reviewed source changes into that checkout before committing. Exclude recordings, generated protocol data, logs, virtual environments, caches and temporary backups from publication.

## 2026-11-11 inference deployment selection

Formal inference requires `--model NAME`. The base robot/runtime settings remain in `configs/inference/default.json`; `configs/inference/deployed_NAME.json` selects the server model identifier, endpoint, SSH or direct TCP transport, wire protocol and `tasks_file`; NAME may be a deployment/checkpoint alias. The referenced inference-owned task file contains instruction, mask and image preprocessing. `instruction_mode=request` sends the configured instruction, while `instruction_mode=server` sends the configured server task identifier and requires the server-reported prompt to match. The offline client test defaults to model `default`. Preserve the existing robot startup, response-mask intersection, q cleanup, periodic execution and server timeout behavior.
