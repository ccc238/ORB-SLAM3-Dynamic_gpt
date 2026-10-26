# 第11批：关键帧和长期地图点准入

本批不改变 B2 残差、B3 融合、B4 阈值、BA 目标函数；不删除历史地图点。
原有原版程序/库不覆盖，修改版只在 `build_yolo11/{bin,lib}` 生成。

## 修改文件清单

| 文件 | 本批变更 |
|---|---|
| `include/Frame.h`, `src/Frame.cc` | N 长度准入数组、启用标记、日志路径；reset/copy/长度检查 |
| `include/KeyFrame.h`, `src/KeyFrame.cc` | 拷贝快照、清理新 KF 关联副本、索引防漏断言、archive v1 |
| `include/MapAdmission.h`, `src/MapAdmission.cc` | 当前状态 MAPPING 策略、索引门控、线程安全汇总、最终双向审计 |
| `include/Tracking.h`, `src/Tracking.cc` | 会话日志路径；初始化及常规 RGB-D 建点前准入 |
| `src/LocalMapping.cc` | 新 KF 观测和三角化两端准入 |
| `src/ORBmatcher.cc` | 普通 Fuse、Sim3 Fuse 的目标索引门控 |
| `src/LoopClosing.cc` | 回环直接增加/替换观测前门控 |
| `src/System.cc` | 旧图缺失语义时拒绝 ACTION 续建；线程退出后审计 |
| `src/Deep/DynamicDetector.cc` | 解锁 MapAdmission，并要求 FinePolicy 开启 |
| `CMakeLists.txt` | 新源文件、独立测试目标；维持隔离输出路径 |
| `tools/map_admission_test.cc` | 合成输入、融合、索引、序列化测试 |
| `tools/check_frontend_mode_configs.py` | 新合法 MapAdmission+Fine 组合；旧非法组合不放宽 |
| `tools/run_frontend_stage_validation.py` | map3 验证标签、精细优化审计和最终地图审计验收 |
| `tools/audit_map_admission_runs.py` | 源码冻结检查、逐入口计数、观测审计汇总、B4 单次更新/不可靠冻结验证 |
| `configs/frontend_modes/rgbd_tum3/mode3_map_admission_validation.yaml` | 新配置，不覆盖旧 YAML，只开启 MapAdmission |
| `docs/MAP_ADMISSION.md`, `docs/DYNAMIC_FRONTEND_STAGES.md` | 实现、边界、实验和复现记录 |

## 配置与时序

新增完整配置：`configs/frontend_modes/rgbd_tum3/mode3_map_admission_validation.yaml`。
复制第10批 fine 配置，只开启已有 `TwoStage.MapAdmission.Enable: 1`。
没有新增阈值/新增 YAML 字段。默认仍为 0。

启用要求 Mode=3、CoarseFilter=1、UseCoarsePoseForB2=1、FinePolicy=1、
FeatureSemantic=1、FeaturePolicy=1、FeaturePolicy.EvidenceOnly=0、B4.EvidenceOnly=0。
B2/B3 保持 EvidenceOnly=1。保留 Pending/Dynamic.AllowMapCreation=0；不允许以这些
保留字段绕过策略。Mode=0/1/2 不允许开启 MapAdmission；旧配置继续走无准入过滤路径。

正常 RGB-D：YOLO/实例关联 → 整图 ORB/标注 → 上一状态粗过滤/粗位姿 →
B2/B3/B4 → 当前状态精细过滤 → TrackLocalMap（二次精细过滤/优化） →
关键帧决策 → 生成当前 MAPPING 快照 → KeyFrame → 深度建点 → LocalMapping。
初始化必须独立建立首张地图，故在初始化 KeyFrame 前也生成快照：新 person=P，
只有背景/已有 S 可建图，不以初始化绕过准入。

## 数据与入口

- Frame 新增 N 长度 `mvAllowMapCreation` 和显式启用标记；reset/copy/assignment 保持对齐。
- KeyFrame 复制语义区域、instance_id、track_id、当前状态、策略、allow 数组；
  不压缩关键点/描述子/地图点索引。禁止项仅在新 KeyFrame 的关联副本中置空，
  不触碰其原有 MapPoint 对象或历史观测。
- `PrepareFrameMapAdmission` 只计算 MAPPING 快照，不改主 Frame 的 MapPoint/outlier/位姿。
- `KeyFrame::AllowsMapCreation(index)` 验证缓存和语义/策略一致；后台只读快照，不更新 B4。
- Tracking：初始化与 `CreateNewKeyFrame` 深度创建前 gate；拒绝项不消耗“最近100点”预算。
- LocalMapping：`ProcessNewKeyFrame` 注册新观测前 gate；`CreateNewMapPoints` 同时 gate 两端索引。
- ORBmatcher：两种 Fuse 都在选择最佳描述子索引前 gate 目标特征，覆盖邻域与 Sim3 融合。
- LoopClosing：直接补回环观测/替换前 gate，防止绕开 Fuse。
- KeyFrame::AddMapPoint / ReplaceMapPointMatch 另有索引级防漏断言。
- 不修改 `MapPoint::AddObservation`；Replace 只转移已有、已准入的观测，由 KeyFrame 索引断言兜底。

BACKGROUND、有效归属的 PERSON_INTERIOR+当前 S+USE_NORMAL 可准入。
P/D/边界/未知/USE_POSE_ONLY 不可创建长期地图点或新增长期观测。
此前准入的历史点随后变动态，不在本批全局追删。

## 日志与审计

每个新会话创建唯一 `map_admission_<timestamp>.csv`。按 stage/KF 聚合：
`map_candidates_total,map_reject_P,map_reject_D,map_reject_boundary,map_reject_unknown,map_points_created`。
candidate 指该入口执行的索引检查次数，含重复访问；不是去重后的 ORB 点数。
created 仅在实际 AddMapPoint 入 Atlas 后计数，且再次断言源特征已获准入。
每一物理新点只在 bootstrap/depth/triangulation_current 中计数一次。
三角化邻居端单独记录，融合只记检查和拒绝，不算创建新点。
CSV 输出持互斥锁，写失败明确终止，不静默丢失审计。

`System::Shutdown` 等待后端线程退出后生成 `map_admission_final_audit.csv`，
遍历所有地图、活动关键帧和地图点双向观测；发现禁止观测或数组不齐时显式报错。
最终审计覆盖保留下来的活动地图；创建时断言另覆盖后来被剔除的新点。
`tools/audit_map_admission_runs.py <batch_directory>` 汇总每次运行并保存 JSON/CSV，禁止覆盖旧审计。

## Atlas 序列化与边界

KeyFrame archive version 升至 1，保存语义准入快照，不存运行日志路径。
读取旧 version=0 时初始化为“禁用准入”的旧兼容语义；B0/B1/B2/B3/B4-A 可继续加载旧图。
开启新 ACTION 地图准入时，不允许续建没有语义快照的旧 Atlas，启动明确报错，
应使用新地图（否则未知旧特征可经融合绕过准入）。新格式 Atlas 不保证可由原版二进制读取。
不得覆盖已有 Atlas 文件。新格式完整会话存取、复杂跨图 merge 和长时回环应另行实测。

## 验证

测试代码 `tools/map_admission_test.cc`：数组/Frame copy、KeyFrame 副本清理、
索引级新增/替换防漏、v1 元数据序列化往返、v0 旧字段载荷读写及默认路径、普通/Sim3 Fuse 拦截、
禁用时原融合、Replace 已有观测转移、坏长度拒绝。
注意“v0 旧字段载荷测试”不等同于完整旧 Atlas 文件迁移测试。

所有构建 `-j2`，重编译 Optimizer 时单独完成其对象，再构建其它目标，避免内存峰值。
先运行 sitting_static/walking_xyz 各100帧（新准入与旧 fine 对照），验收后才能运行全序列。
运行产物保存生效配置、commit、命令、日志、B2/B3/B4 CSV、两条轨迹和准入审计。
本批不将编译/跟踪成功等同于 ATE/RPE 或动态语义正确性证明。

## 手动复现（真实本机路径）

下例完整 walking_xyz，最后两个参数分别为 Viewer=1、按时间戳播放=1。
无显示环境用 `0 0`，不要用 offscreen 后端强行启动 Pangolin 窗口。
使用 `mkdir`（不加 -p）保证结果目录已存在时明确失败，且不要在旧目录重复运行。

```bash
(
set -euo pipefail
repo=/home/aaa/ORB-SLAM3_original0
run_dir="$repo/results/manual_map_admission_walking_$(date +%Y%m%d_%H%M%S)"
mkdir "$run_dir"
cd "$run_dir"
cp "$repo/configs/frontend_modes/rgbd_tum3/mode3_map_admission_validation.yaml" effective_config.yaml
git -C "$repo" rev-parse HEAD > commit.txt
export LD_LIBRARY_PATH="$repo/build_yolo11/lib:$repo/Thirdparty/DBoW2/lib:$repo/Thirdparty/g2o/lib:$repo/third_party_ort/onnxruntime-linux-x64-1.17.3/lib:${LD_LIBRARY_PATH:-}"
"$repo/build_yolo11/bin/ablation_sequence" rgbd \
  "$repo/Vocabulary/ORBvoc.txt" \
  "$run_dir/effective_config.yaml" \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt \
  0 1 1 2>&1 | tee run.log
)
```

自动验证仍以 `tools/run_frontend_stage_validation.py` 为准：它会额外保存配置快照、commit、
完整命令、退出码和轨迹行数校验，Viewer=0，无需 GUI。

## 本机已完成的第11批验证（2026-10-23）

- 基准 commit `34c523ee399af8c1ef82721b5fbf64f64618a76e`，分支 master1。
  用户原有工作区修改保留；没有提交 Git。原程序/库 SHA256 前后相同。
- 构建/冻结目录：`results/frontend_stage_validation/batch11_map_admission_build_01`。
  含 pre_batch11_sources.tar.gz、相对冻结版补丁、构建日志、单元测试、原版/修改版二进制哈希。
  `git diff --check` 的全仓报告仅涉及 CMake 生成的 link.txt 尾空格；源码范围检查通过。
- 所有目标使用 -j2，Optimizer 对象单独先完成；10 项单元测试通过，另补测 v0 载荷读写通过。
  RGB-D、stereo_euroc、mono_tum 以及验证目标编译通过。
- YAML：48 个合法/非法组合检查全部通过，结果在
  `results/frontend_stage_validation/config_checks_20261023_095730`。
- 先通过新准入/旧 fine 两模式、两序列各100帧，共4个短片段，目录：
  `results/frontend_stage_validation/batch11_map_admission_short_20261023_095742`。
- 完整新准入模式：`results/frontend_stage_validation/batch11_map_admission_full_20261023_095948`。
  `summary.json`、`map_admission_summary.csv`、`map_admission_audit.json` 是主验收文件。
- 旧 Mode0/Mode1/Mode2/旧 B4-A 各在两个序列运行100帧，共8个兼容性短片段全部通过，
  均100/100帧 OK、exit=0、LOST=0。目录：
  `results/frontend_stage_validation/batch11_legacy_compatibility_20261023_100546`。
  关闭 MapAdmission 时均未生成准入 CSV，旧 fine 对照亦未生成。

| 序列 | 输入/OK帧 | 相机/关键帧轨迹行数 | exit / LOST | 实际新建点（全运行累计） | 禁止观测/错位 |
|---|---:|---:|---:|---:|---:|
| sitting_static | 680/680 | 680/7 | 0/0 | 2083 | 0/0 |
| walking_xyz | 827/827 | 827/203 | 0/0 | 32851 | 0/0 |

| 序列 | candidates检查 | 拒绝P | 拒绝D | 拒绝边界 | 拒绝未知 |
|---|---:|---:|---:|---:|---:|
| sitting_static | 37499 | 8696 | 0 | 2739 | 0 |
| walking_xyz | 4730358 | 209604 | 39468 | 113412 | 272 |

以上是各入口重复检查的累加，不能当作去重的特征数量。新点只在真实创建后记录；
创建入口断言和最终双向审计共同验证 P/D 新建点=0。最终活动 KF 为7/203，
检查正向观测1648/35492、反向观测1631/33863，禁止观测均为0。
历史融合/剔除导致累计新点数不同于最终保留点数。

sitting：B3 MOVING=0，B4 S/P/D=1330/30/0 实例帧；
walking：B3 MOVING=34，B4 S/P/D=220/449/351 实例帧。
walking 转移：P_TO_S=7、S_TO_P=5、P_TO_D=5、D_TO_P=4；
sitting：P_TO_S=2，其它上述转移为0。
B4 每帧每轨迹只更新一次、不可靠证据冻结计数的 CSV 审计通过。
B2/B3/B4、MapPoint、Optimizer、FineFeaturePolicy、FeaturePolicyEvaluator 源文件与冻结版一致；
Tracking::Track/TrackLocalMap/NeedNewKeyFrame/重定位等函数逐函数一致。

未验证：ATE/RPE 精度、GUI 实时展示、真实双目/单目序列、新/旧完整 Atlas 续建、
复杂长时回环和多图 merge。合成测试覆盖两种 Fuse，但不能代替真实回环验收。
不承诺修改版禁用模块后与独立原版逐帧轨迹、时延完全相同。
