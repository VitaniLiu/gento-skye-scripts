# Gento Skye LeRobot v2.1：convert 与 clean

2026-11-05 更新。两个独立阶段，convert 不因 action/state 缺失删帧；clean 按静止帧文档生成待人工审核的删帧候选。旧的“转换即筛掉部分缺失帧”规则废止。

## 目录与入口
正式转换：/mnt/apex-data/gento-skye_data/lerobot/<任务名>/
独立清洗：/mnt/apex-data/gento-skye_data/lerobot_clean/<任务名>/
每个任务共用data和meta，data/chunk-000/episode_000000.parquet等，每1000条切换chunk。训练Parquet仍为固定11列。
convert入口 scripts/processing/convert；clean入口 scripts/processing/clean。交互bash通过deployment/host/convert-shell.bash提供同名函数。新终端自动加载；旧终端source此文件。
clean启动后交互选择本地lerobot任务，再选择全部episode或单条episode。每次从lerobot重建所选任务的clean输出，未选中episode按原样保留；不累积清洗，不修改源数据。任务写入锁与convert共用。
任务腰高配置在configs/processing/task_export_v21.json，test和当前shopping均开放；shopping任务设计本应锁腰，本次开放用于保留误操作产生的真实数据。

## convert：完整保留
只要图像可解码并能建立对齐帧，即使所有state/action都缺失，也写入数据。缺失数值填0，不拿反馈替代动作，不用未来消息回填，不伪造原始文件中没有的图像。
头部图像时间范围定义20Hz网格；解码失败或图像缺失超过保持区间无法恢复，必须保留异常报告，不能声称还原未录到的帧。
同一原始episode的多个输入片段合并为一个输出episode，不因动作缺失拆分。输出frame_index与timestamp在episode内从0开始；index跨episode连续。
训练Parquet字段：observation.state、action、validity、observation.images.head_color、observation.images.left_wrist_camera、observation.images.right_wrist_camera、timestamp、frame_index、episode_index、index、task_index。
state/action=float32[78]，validity=uint8[78]结构掩码；真实零动作与缺失占位零不同。
腕部相机不存在，两个字段为nullable struct<bytes:binary,path:string>且逐帧null；头部原生RGB/PNG字节内嵌，shape元数据CHW。
meta基础文件：info.json、episodes.jsonl、tasks.jsonl、episodes_stats.jsonl、stats.json、schema78.json、training_schema.json、export.json。
新增meta/frame_quality.parquet：逐帧state/action有效性、原因、来源时间、header时间和样本年龄；meta/source_frame_mapping.parquet：源任务、episode、帧序号、绝对网格及相机时间。
训练必须同时应用结构validity和逐帧动态有效性。数值stats保留存储占位0，不能直接把缺失值纳入训练归一化估计。
旧版已过滤数据缺少sidecar时必须重转；不根据数值是否为0猜测历史有效性。

## 78维语义
base_link XYZ(m)+rotation6d前两列 [R00,R10,R20,R01,R11,R21]，从内部前两行编码恢复完整矩阵再转换；缺失旋转保持0且动态掩码无效，不伪造单位姿态。
27腰旋转和28腰前后锁死，输出0且结构无效；29腰上下，上为正，按任务配置；30底盘前后，前为正；31底盘旋转锁死，state/action输出0且结构与动态掩码均无效。shouwu任务29腰上下同样锁死；身体仅30底盘前后启用。
32:54、55:77灵巧手不用，填0且结构无效；54、77真实夹爪[0,1]。
state/action保持同一行各自来源，无下一帧移位。

## 读取与验证
src/processing/task_v21.py提供load_training_dataset(root, with_quality=True)，返回HF数据集、质量表和源映射表；默认只返回11列数据集。训练变换仅启用head_color。
PyArrow与HF Image解码已验证；安装的lerobot0.4.3读取v3，不宣称旧版原生读取器对null相机扩展已验证。
现行清洗验证为 tests/static_clean_update/test_static.py；旧 clean_quantile、convert_clean_update 中的分位删帧断言属于历史规则，不再适用。
测试和缓存全部在项目tests/。原始MCAP仅在机器人通过SSH只读，不复制。录制完整性、物理时钟同步和CRC的已有验证边界保持如报告所述。

## 静止帧 clean（2026-11-05，替代缺失段分位数规则）

入口仍为 `clean`：选择任务及 episode，先生成审核包，再选择是否写入 `lerobot_clean/<task>` 候选。新增 `clean --list` 与 `clean --preview TASK`；后者只读且在生成前后核验源文件哈希，不占转换写锁。正式 clean 仍持有转换锁。

筛选依据 static_frame_filter_logic.pdf：20Hz 原始相邻帧，对 state/action 独立计算位姿、手关节及腰旋转速度；所有有效通道严格低于阈值。6D 前两列重建正交旋转矩阵后计算相对角。连续静止严格超过0.25秒，两端各保留3帧；拼接端点必须同时满足 state/action 差值上限。无效数值、动态掩码缺失、时间断点及 episode 边界保留。夹爪与腰升降要求数值完全不变，底盘速度必须为零；这属于 Gento 额外保守保护，不套用手指关节单位。结构无效维度不参与。

速度阈值（严格小于）：位姿平移0.005m/s，旋转0.05rad/s，手关节最大速度0.04rad/s，腰旋转0.05rad/s。拼接差值上限（允许等于）：平移0.001m、旋转0.01rad、手指及腰旋转0.02rad。

目标保持/补齐保护来自源 `meta/target_protection.json`，格式：
```json
{"episodes":{"0":{"complete":true,"source":"已核实的录制/转换来源说明","ranges":[{"start":100,"end":120}]}}}
```
区间为原始 episode 帧号半开 `[start,end)`，自动向前后各扩展1帧。只有来源经过核实并覆盖整条 episode，才可标记 complete；空 ranges 不能凭空表示没有保持区间。现有旧导出缺少此信息，默认 complete=false，展示低速曲线与区间但不删帧。不要为了启用筛选伪造完整性。

输出使用唯一 keep_indices 选择全部行与质量/来源侧表，不改写运动值和图像字节、不插值、不拆 episode。重建索引、20Hz 压缩时间轴及统计；逐字段检查保留内容，再检查源哈希未变。输出 clean_report 的 review_status 始终为 unapproved_candidate，人工复核后才能用于正式训练。不会自动晋级为正式数据。

审核包 `conversions/<id>/static_frames.html` 含原视频、候选输出视频、候选±1秒上下文、逐张待删帧、symlog 无平滑速度曲线和映射。视频逐条解码验证20Hz及帧数。无腕部相机明确标注不存在。原视频与曲线按源帧联动，筛后/上下文独立播放。网页只读，不提供批准或删除操作。

Windows 本地入口为研究工作区 `查看clean预览.cmd`：选任务后自动生成、下载审核媒体，通过仅监听127.0.0.1且支持Range的本地HTTP服务打开网页。关闭服务可按启动窗口提示 Stop-Process；无需复制原始数据或LeRobot数据集。依赖项目 .venv 的 av/libx264 与 matplotlib，缓存位于 tests/cache。

## 逐条检查、转换和续跑（2026-11-06）

入口仍为 `convert`，选择“检查并转换”和范围即可。整个目录及仅未转换两种模式都会逐条核对记录；已经完成且未变化的自动跳过，单条模式也采用相同的安全复用规则。原始采集发生变化才重新处理，不再依赖本轮结束后的总报告来判断是否保存。

处理顺序为：来源轻量指纹 → 查询检查点 → 检查或复用检查 → 转换本条 → 完整验证本条 → 持久保存检查点 → 提交任务目录 → 登记完成 → 释放内存与清理本条派生PNG等临时工作文件。任一条失败会记录原因并继续其他条；中断重启从已保存检查点继续。

来源指纹包含文件列表、大小、纳秒修改时间和采集提交状态；检查及转换另有代码/映射/任务配置指纹。旧检查观察在来源一致且属于本次审查过的原始检查版本时可以导入，再执行当前语义复核。正式输出已有SHA-256与文件签名；签名变化后重新核验内容。转换规则变化不能凭旧的完成标记跳过。

`conversions/checkpoints/<task>/<key>/record.json` 保存 inspected、ready、committed 状态，dataset保存单条已验证结果及动态质量/来源信息。`conversions/task_transactions/<task>.json` 保存任务提交日志，启动时修复中断的提交。旧任务版本位于conversions/commits/previous_<id>，不删除原始或已验证数据。

每完成一条，`lerobot/<task>` 就是完整可读的任务数据集，仍为data和meta、固定11列、不拆episode。未变的历史Parquet硬链接复用，不重新加载图像。只重新写入新增/替换条及受全局索引变化影响的数据文件；质量表和来源映射按批次汇总，数值统计和精确分位数通过磁盘暂存计算，不在内存积累历史图像。单条全量验证后再核对任务元数据与索引；历史文件签名未变时不重复解码图像。

“释放历史行”是释放内存中的已提交图像/数组/Arrow对象，不删除机器人MCAP、任务Parquet或检查点。只清理本次成功提交所对应的派生工作目录；失败结果保留用于恢复。

本次核查发现旧批处理的101个可读Parquet缺少info/export及动态质量、原始帧映射，不能直接当作完整检查点。已保存恢复审计清单tests/incremental_convert/recovery_audit.json并保留全部旧文件。匹配的历史检查可复用；缺失的逐帧证据必须重建，禁止按数值是否为0猜测有效性。

验证脚本tests/incremental_convert/test_incremental.py模拟检查后、转换后、提交准备后、旧目录移走后、新目录安装后的终止，验证续跑不重复转换、来源/代码变化失效、整行图像不变、替换后的全局索引及完整task_v21读取。


### Clean 分批存储（2026-11-08）

运行 scripts/processing/clean，选择任务/episode 并在预览后输入 y。清洗每次仅加载一条 episode，核对保留字段、图像字节和 sidecars 后释放内存。每 10 条（末批可不足 10 条）在 conversions/clean_checkpoints/<task>/<key>/ 持久保存；终端分别显示处理/已保存数量。批次目录是恢复产物，不是完整训练数据集。全部完成后流式重建全局索引、sidecars、磁盘统计并统一生成 clean_report.json，再通过可恢复事务发布 lerobot_clean/<task>。原数据及之前的清洗结果保留。

中断后可运行 scripts/processing/clean --resume-report /mnt/apex-data/gento-skye_data/conversions/conversion_... 。该参数复用此前确认的清洗计划与审核媒体，源数据、选择或相关代码改变时不得复用旧批次；校验失败时停止。已经完成的匹配批次不再清洗，未提交批次重新处理。检查点当前保留用于恢复，会额外占用磁盘。
