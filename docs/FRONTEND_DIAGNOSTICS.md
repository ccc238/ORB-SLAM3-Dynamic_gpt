# 第13批：统一前端日志与 Viewer

## 范围与开关

本批只读采样，不改变 B2 残差公式、阈值、B3 规则、B4 状态转移、匹配器、优化器、关键帧决策或地图准入策略。
旧 YAML 无新增字段时，三个开关均为 `0`：

```yaml
DynamicFrontend.Debug.SaveCsv: 0
DynamicFrontend.Debug.ShowOverlay: 0
DynamicFrontend.Debug.SaveOverlay: 0
```

- `SaveCsv`：RGB-D 统一日志。在线写 `dynamic_frontend_online.csv`，正常 Shutdown 等待后端线程结束后写 `dynamic_frontend.csv`。
- `ShowOverlay`：FrameDrawer/Viewer 显示当前主 Frame 特征状态及独立动作标签。
- `SaveOverlay`：无需 GUI，调用同一个 FrameDrawer 绘图函数，每100帧保存一张 `dynamic_frontend_overlays/frame_<id>.png`。首帧编号为0。与交互显示独立。
- `ShowOverlay=0, SaveOverlay=1` 时在私有图像上调用纯绘图 helper，不切换任何共享Viewer标志；原B2/B3/B4显示开关仍各自控制旧显示，不由新增开关擅自关闭。
- 任一开关开启才采集新增阶段耗时；正式测速三者均关闭。日志、绘图和 PNG I/O 会改变耗时与线程调度，不能用诊断模式的运行速度代表关闭诊断时性能。
- 纯算法测速还应按既有开关关闭B2/B3/B4、FeaturePolicy的CSV/终端调试，以及YOLO预览/保存；只关闭这三个新增开关不等于关闭所有旧调试输出。本批验证配置刻意保留原日志供交叉核查。
- 配置读取沿用 `DynamicDetector`，严格检查0/1。旧配置、原始可执行文件及库不覆盖。结果输出位置沿用 `B2.Debug.CsvOutputParent`，应每次切换到独立工作目录。统一文件已存在时明确报错。

完整验证配置：`configs/frontend_modes/rgbd_tum3/mode3_diagnostics_validation.yaml`，从上一批 MapAdmission 验证配置复制，仅增加上述输出开关。全部相机、ORB、B2/B3/B4及两阶段参数保持不变。
另有 `mode3_diagnostics_save_only_validation.yaml`，仅将新增 `ShowOverlay` 改为0，用于验证保存/显示独立性。

## 调用顺序

RGB-D输入 → YOLO → 统一实例关联 → B2独立候选 → 主Frame ORB（可选计时）→ 语义及历史状态标注 → 原始初始跟踪分支（粗位姿计时）→ 原粗位姿可靠性门控 → B2 Process → B3 → B4 → 当前精细策略 → TrackLocalMap（精细阶段计时）→ 最终位姿/内点 → 原关键帧决策及地图准入 → 旧B3/B4日志 → **只读统一快照** → FrameDrawer复制快照 → 可选离屏PNG。

Shutdown：等待 LocalMapping / LoopClosing / GBA（及适用的 Viewer）结束 → 原地图准入审计 → **按来源关键帧的 frame_id 汇总地图事件并生成最终统一CSV**。未正常Shutdown时保留在线CSV，不能把它当作已完成的建图统计。

## CSV口径（必须按 row_type 分组分析）

统一列包括任务要求的全部字段，另外增加 `row_type`、`feature_count`、三个 action、`coarse_attempts`、地图统计范围/待完成标志及生效开关等。

| row_type | 含义 | 可汇总字段 |
|---|---|---|
| FRAME | 每输入图像一行 | 阶段匹配/拒绝/内点、建图事件计数、回退及耗时 |
| INSTANCE | 每个当前person或TTL历史track一行 | B2/B3/B4证据、状态计数及**人体内部**策略 |
| FEATURE_GROUP | 主Frame正式ORB按区域/ID/前后状态/策略分组 | feature_count，各组和严格等于Frame::N |

禁止跨三类行直接求和。实例行与特征组行不复制帧级计数，避免多个人导致重复统计。
`semantic_region` 在 FEATURE_GROUP 中为真实 `BACKGROUND/PERSON_INTERIOR/PERSON_BOUNDARY/UNKNOWN`；INSTANCE 行使用 `PERSON_INSTANCE/NOT_VISIBLE`；FRAME 行为 `FRAME_SUMMARY`。实例行的策略只表示人体内部，不能据此宣称边界点被允许。

- B2/B3/B4依据 **同一帧+instance_id+track_id** 关联。TTL丢失行不沿用旧帧几何结果，不伪造B2数值。终止轨迹可写CSV，但超过TTL后不再绘制。
- 空字段表示不适用/未采集；负残差值继承B2“无有效值”约定，不能当真实残差。阶段耗时 `-1` 表示未执行，不能当0参与均值。
- `allow_coarse/fine/map` 是**策略资格**。粗策略用上一状态，精细及地图用当前状态。状态改变时它们可以不同。未标注的旧模式不计算特征策略，资格字段留空，动作标记 `DISABLED`。
- `coarse_action/fine_action/map_action` 区分 `PROPOSE_*`（模块没应用的建议）、`PASS/REJECT/DENY`（已启用的策略）、`NOT_RUN`、`RESTORED_FALLBACK`、`NOT_VISIBLE`。地图动作表示准入资格，**不是承诺本帧插入关键帧**。
- coarse计数取最后一次原始初始跟踪分支调用，`coarse_attempts`记录尝试数，不把两分支匹配数相加。未应用过滤时不伪装为REJECT。若发生回退，保留原粗过滤CSV中的恢复前后细节；统一的过滤计数仍是该次过滤尝试的计数。
- fine计数取 `SearchLocalPoints()` 后、PoseOptimization前的第二次过滤，最终内点从该次 TrackLocalMap 结果采样。第一遍过滤仍保存在旧 `fine_feature_policy_*.csv`。
- 地图六项统计从旧 `map_admission_*.csv` 按**来源KeyFrame的frame_id**汇总，仅写FRAME行，不归到处理时并发输入的“当前帧”。它们是各准入位置的事件/检查次数，不是互斥特征数；不支持伪造逐实例建图数量。未启用地图模块时留空。在线表标记pending，最终表清除pending。
- 所有旧独立CSV保留，可交叉核查。`dynamic_frontend_schema.json`记录上述范围和单位。

## 耗时定义（毫秒，steady_clock）

- yolo_ms：沿用检测器的推理耗时。
- orb_ms：主RGB-D Frame::ExtractORB，包括现有提取统计复制，**不包括B2候选ORB**或整个Frame构造。
- coarse_ms：原始初始跟踪分支直到共同粗位姿边界，不含B2/B3/B4、局部地图精细阶段。
- b2_ms：显式Process调用，不含候选提取与背景快照准备。
- b3_ms、b4_ms：各自现有函数调用，不增加第二次状态更新。
- fine_ms：整个TrackLocalMap，含局部地图更新、搜索、过滤、优化与检查。

这些值不是整帧总耗时的完备分解；未包含所有预处理、日志、候选提取和关键帧操作，不能相加冒充端到端耗时。

## Viewer约定

背景绿色，人体S青色，P黄色，D红色，边界/未知灰色；TTL内丢失框灰色。新模式下旧B2候选色环/旧B3框标签不覆盖主特征状态色。
实例框附近单独显示 `B4 state=...`、`coarse_action=...`、`fine_action=... map_action=...`、`Ns/Nd/Nr`与转移原因。
红色表示B4状态，不把B2高残差直接标成Dynamic。B2原始证据仍保留日志/顶部文本。
绘图只接收值快照；无策略计算、状态更新、MapPoint写操作。GUI关闭也能保存相同绘图函数输出。

## 复现与验收

```bash
cd /home/aaa/ORB-SLAM3_original0
python3 tools/run_frontend_stage_validation.py \
  --stage batch13_diagnostics --short-only --require-main-person \
  --mode-config kf3=configs/frontend_modes/rgbd_tum3/mode3_diagnostics_validation.yaml
```

先对两段100帧做 `tools/audit_frontend_diagnostics.py` 审计，再用runner的 `--short-reference <短测批次目录>` 新建完整序列结果，不能覆盖短测。
runner保存运行命令、配置快照、commit、退出码、轨迹、旧CSV与日志。
新单测 `frontend_diagnostics_test` 检查颜色、离屏绘图、CSV转义、禁止覆盖、地图事件跨帧归属及计时。源码审计验证冻结算法未改。

头文件改变后先单独编译Optimizer，结束后再用 `-j2` 编译其他目标。结果及最终验收记录见本批结果目录。编译/日志审计不是ATE精度验证，离屏PNG不是交互Pangolin验证。

## 后续风险

尚未声称关闭诊断后的修改版与原版逐帧轨迹或速度相等；多线程调度存在自然差异。发生真正丢失/重定位/新图与粗精失败的长序列仍需专项验证。
地图统计按来源帧归档，不提供每个实例的异步MapPoint创建数；若未来需要必须从具备索引上下文的MapAdmission事件扩展，不能在通用MapPoint::AddObservation里粗暴处理。
当前日志用于单次连续会话；未来支持重置后复用frame_id或拼接多个会话时，应增加session/map标识再联接。

Frame新增计时字段及RGB-D构造函数可选参数，源码调用默认兼容，但旧目标文件与新共享库不能混用；头文件变更后必须重编依赖目标。本批保留了第一次过早启动旧测试程序造成的符号查找错误，完成全量目标编译后已在新目录重跑。使用同一批 `build_yolo11/bin` 与 `build_yolo11/lib`，不要将原版可执行文件指向修改版共享库。

旧终端日志可能被后端输出穿插。验证脚本在统一CSV存在时使用FRAME快照核查前端标志，旧结果无统一CSV时保持原解析行为；没有降低关键帧顺序、最终内点或地图准入的断言。

## 文件职责

| 文件 | 本批修改 |
|---|---|
| include/Deep/DynamicFrontendConfig.h | 三个独立输出开关、是否采集新增计时的只读条件 |
| src/Deep/DynamicDetector.cc | 在原解析模块读取/校验并打印开关 |
| include/FrontendDiagnosticLog.h | RAII计时器、阶段计数值、统一日志接口 |
| src/FrontendDiagnosticLog.cc | CSV schema、转义、禁止覆盖、后端结束后的来源帧聚合 |
| include/Deep/FrontendDiagnostics.h | 纯值的特征状态、实例状态动作和耗时显示快照 |
| include/Tracking.h | 日志/计时成员与采样、收尾接口 |
| src/Tracking.cc | 现有调用边界计时、匹配计数采样、发布快照及可选离屏PNG；不新增匹配/优化/状态更新 |
| src/TrackingDiagnostics.cc | FRAME/INSTANCE/FEATURE_GROUP只读输出、实例同帧关联、动作与状态分离 |
| include/Frame.h、src/Frame.cc | RGB-D主ORB可选计时、拷贝计时值，旧调用缺省不计时 |
| include/FrameDrawer.h、src/FrameDrawer.cc | 主ORB状态颜色、独立动作标签、独立图例/耗时页脚、值快照离屏绘制 |
| src/System.cc | 后端线程join之后完成日志归档 |
| CMakeLists.txt | 新源文件和新单测；继续隔离所有输出 |
| tools/frontend_diagnostics_test.cc | 配色/绘图/转义/禁止覆盖/异步地图归属与计时单测 |
| tools/run_frontend_diagnostic_unit_tests.py | 新目录执行12单测及9个新增开关检查 |
| tools/audit_frontend_diagnostics.py | 冻结源码、分组数量、CSV关联/计数/耗时/图像及关键帧审计 |
| tools/audit_keyframe_order.py | 有统一CSV时优先使用前端快照，不再依赖被穿插的终端行 |
| tools/run_frontend_stage_validation.py | 同样优先使用结构化帧级计数，保留旧配置/无统一CSV路径 |
| configs/frontend_modes/rgbd_tum3/mode3_diagnostics_validation.yaml | 从旧配置完整复制，只增加三个输出开关 |
| docs/FRONTEND_DIAGNOSTICS.md | 本文：口径、开关、复现命令与限制 |

## 本机最终验收（2026-10-23）

最终运行库SHA256：`875a0ef22582cce3d2966a961ff227f1e5f69e065eac690d231b9383fdfead4e`。
原版可执行文件和原版库的SHA256与本批前一致，未提交Git。

- 编译：Optimizer单独完成后，其余目标使用`-j2`；RGB-D、双目、单目示例及12个单测目标完成。
- 单测：最终`unit_05`的12项单测通过；48项原配置检查、9项新增输出开关检查通过。
- 短测：`batch13_output_independence_short_20261023_145727`，显示+保存、仅保存两配置，各对两个序列跑100帧，4项通过并完成CSV审计。
- 兼容：`batch13_legacy_compatibility_20261023_145315`，旧Mode0/1/2和旧B4-A各跑两个100帧片段，8项全部OK且未生成新增诊断文件。这组运行在最后的保存/显示独立性修复前完成；最终补丁仅改变启用SaveOverlay的分支，默认关闭分支未变。各运行保留真实运行库哈希，不将不同构建冒称同一构建。
- 最新完整：`batch13_final_independent_full_20261023_145935`，在最终运行库上重跑两序列。该批为正式完整结果；早先完整结果保留但不混作同一次运行。

| 序列 | 输入/OK | Camera行数 | KeyFrame行数 | fine拒绝P/D | 最终地图禁入观测违规 | PNG |
|---|---:|---:|---:|---:|---:|---:|
| fr3_sitting_static | 680/680 | 680 | 7 | 92/0 | 0 | 7 |
| fr3_walking_xyz | 827/827 | 827 | 153 | 4983/4909 | 0 | 9 |

两段完整运行退出码均为0，RECENTLY_LOST/LOST均为0；每帧特征分组数和N一致，B2/B3/B4同帧身份及数值与旧CSV一致，P/D进入精细优化数量均为0，地图事件来源帧归档逐项核对通过。
已检查合成配色图、sitting的S状态图、walking的D状态图以及保存独立分支。没有启动交互式Pangolin，因此不宣称实际桌面窗口验收通过；没有做本批ATE/RPE、真实双目数据精度验证或特殊丢失/重定位回归。

构建/测试/冻结/最终JSON：`results/frontend_stage_validation/batch13_diagnostics_build_01/`。
最终完整运行及CSV/轨迹/图片：`results/frontend_stage_validation/batch13_final_independent_full_20261023_145935/`。
