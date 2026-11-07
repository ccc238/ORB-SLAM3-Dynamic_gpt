# B2 三维运动证据可靠性增强

## 基础版本与范围

基础提交 `968bacd072c4af2912f6660020a8410e8e3964b0`，分支 `snapshot_clean`。保留工作区此前的多类别扩展，不自动提交、推送或回退用户改动。

本轮不引入候选级光流、临时身份延续、不修改后端BA、不删除历史地图点。原三维残差、背景median/MAD、深度分箱、实例聚合与R/E/P判定保留；增强下背景参考按MapPoint ID拒绝重复采样，不改变MAD公式。B4的3/6/10、TTL=15、有效期间隔0.20s及S有效期0.30s未调整。

## 原有检查与补齐内容

原有：ORB Hamming距离、双向各自Lowe ratio（须有第二邻居）、可配置互检、几何配对权限及关联帧号/时间戳检查；原始跟踪成功/初始化/重定位/恢复状态、粗位姿内点数及比例、背景数量；B2样本数量/比例/背景参考门控。

缺项：B2的nearDepthEdge固定false，深度中位数允许过少样本；关闭互检后可能复用当前候选；粗位姿重投影误差混合目标和背景且仅记录；未检查背景分布；点级与实例级拒绝诊断不全。

补齐：实例内深度邻域质量、显式一对一匹配去重及候选ID检查、可信且相邻的身份/测量配对、背景专属重投影与空间覆盖门控、数值检查及结构化诊断。增强关闭时保留旧算法分支，不把新增门控隐式应用于旧配置。

## 配置

```yaml
B2.Reliability.Enable: 0
B2.Reliability.Depth.Radius: 1
B2.Reliability.Depth.MinValidSamples: 5
B2.Reliability.Depth.MinValidRatio: 0.60
B2.Reliability.Depth.AbsoluteMeters: 0.05
B2.Reliability.Depth.Relative: 0.02
B2.Reliability.Coarse.MinBackgroundInliers: 30
B2.Reliability.Coarse.MinBackgroundInlierRatio: 0.30
B2.Reliability.Coarse.MaxMedianReprojectionPixels: 2.5
B2.Reliability.Coarse.GridCols: 4
B2.Reliability.Coarse.GridRows: 3
B2.Reliability.Coarse.MinGridCoverage: 0.25
```

Enable默认0。深度半径、绝对/相对阈值若未单独配置，复用`FeatureSemantic.DepthEdge.*`；背景数量/比例若未配置，复用现有`TwoStage.MinCoarseInliers/MinCoarseInlierRatio`。新增值是工程调试初值，不是已标定最优值。统一在DynamicDetector读取、校验并打印；不另建YAML解析器。

开启要求已有B2、Geometry和Decision均开启，并使用`TwoStage.UseCoarsePoseForB2=1`。非法范围、NaN和无效依赖明确报错。

完整配置：

- `configs/b2_reliability/rgbd_tum3/enhancement_off.yaml`
- `configs/b2_reliability/rgbd_tum3/enhancement_on.yaml`

两者仅Enable一行不同，其他相机、ORB、A3、多类别、R/E/P、B3/B4、粗精策略和地图准入设置相同。不要用旧`build_multiclass`程序测试新开关；新程序位于`build_b2_reliability/bin`。

## 点级深度检查

深度输入为Tracking已转换为米的CV_32FC1图。对每个独立B2候选，在其原始实例掩膜内部、图像范围内采样：

1. 忽略非正、NaN及无穷深度。
2. 同时检查有效数与有效比例。比例分母是邻域内、实例掩膜内且在图像内的像素数。
3. 使用原有上中位数约定得到深度，检查现有MinDepthMeters/MaxDepthMeters。
4. 有效深度最大值减最小值若严格大于`max(AbsoluteMeters, Relative*medianDepth)`，判DEPTH_EDGE。

默认3×3完整邻域需同时满足数量>=5、比例>=0.60，实际至少6个有效像素。中心深度缺失但周围支持充分且表面一致时允许中位数补偿；不是所有中心孔洞都无条件拒绝。上一帧和当前帧任一端质量不通过，匹配保留诊断记录但不参与三维聚合。不删改主Frame的关键点、描述子或索引。

采用max而不是abs+rel，是为了复用现有Frame深度边缘阈值约定。Frame现有检查仅共享阈值函数，原有行为不变。

## 关联、匹配和数值

增强模式要求关联已评估、允许状态继承、允许几何配对、状态为CONTINUOUS_CONFIRMED，实例/轨迹/类别一致；关联记录指向B2实际保存的上一帧，帧号相邻、时间严格递增且历史时间戳一致。不接受重关联首帧或跨多帧观测套用相邻帧位姿。

复用原有Hamming/Lowe/互检及阈值。关闭互检时也不能重复占用当前特征；按原查询顺序保留首个通过原门控的对应，其余标记MATCH_DUPLICATE_TARGET。重复候选ID令该实例不可靠，避免查找时索引歧义。

有限性检查不以残差“大”为理由。只拒绝无效深度/像素/投影、NaN、无穷及无效参考统计。真实运动的大r3d和大r2d可保留；没有添加针对目标的小二维残差门槛。

## 粗位姿质量：时序、背景与分母

保持原正常运动模型、参考关键帧、重定位、初始化和Atlas控制流。新增诊断在原粗位姿分界点执行，先于B2与TrackLocalMap；不使用精细优化结果回填B2。

可信背景规则：在当前全部受管理类别的原始掩膜及既有膨胀带之外；MapPoint未坏且有长期观测；深度有限并在原有效范围内。粗优化前记录这类**已匹配特征数**作为背景内点比例分母；粗优化后且非离群、有有限正深度投影的背景对应作为分子和质量样本。不将优化后幸存数自身当分母。

- 重投影：世界MapPoint经粗Tcw及相机模型投影，与`mvKeysUn`去畸变坐标比较，单位pixel，取中位数。
- 掩膜/覆盖：用`mvKeys`原始图像坐标查原始实例掩膜并分配网格。
- 覆盖率：至少有一个可信背景内点的格数 / 全部格数。默认4×3，0.25表示至少3格；不是除以背景可见格数。
- B2背景MAD配对仍使用其原几何像素；增强模式额外保存原图maskPixel，避免用去畸变坐标查原图掩膜。

原有混合重投影字段保留用于对照，但新增门控只使用背景专属字段。原失败回退调用顺序不变，新增质量失败通过已有poseReliable=false路径阻止B2/B3/B4可靠证据，并沿用原关键帧防护逻辑。

当前B2独立候选仍采用原针孔K反投影；本轮TUM3零畸变/已校正RGB-D验证不等同于已适配非零畸变、鱼眼、单目或双目前端。

## 判定与状态机

顺序：帧级位姿 → 身份/配对 → 对应点测量 → 实例数量/比例/背景参考 → 原三区域判定。

`R=median(raw_r3d)`；`e=max(0,r3d-m_bg)/sigma_bg`；`sigma_bg=max(1.4826*MAD,原SigmaFloorMeters)`；`E=median(e)`；`P=count(e>=原Decision.MovingThreshold)/valid_count`。不更改任一原阈值。

不可靠或UNCERTAIN只输出证据不足，B3不可靠，B4不增加Ns/Nd/Nr。仍每帧每轨迹调用一次原B4 Process，执行证据有效期和TTL；旧S可因超时转P，D不会因缺证据自动恢复。不能把“冻结证据更新”理解成冻结图像采集时间。

## 日志和原因码

原B2 CSV字段保留并追加列，按字段名读取，不按固定列号。

- 点级`b2_candidate_features.csv`：保留坐标、深度、descriptor_distance、r3d、normalized、r2d和reason_code，追加track_id、previous_candidate_id。valid指该点三维/归一化测量有效，不等同于实例可推动B4。
- 实例`b2_instance_evidence.csv`：追加class_id、depth_invalid_count、depth_edge_count、match_invalid_count、duplicate_match_count、association_invalid_count、numeric_invalid_count、pose_background_sample_count、background_reprojection_median_px、background_grid_coverage、pose_quality_reason、geometry_reliable、evidence_update_blocked、quality_sampling_ms、reliability_enabled。
- `coarse_pose_quality_*.csv`：追加优化前背景匹配数、质量样本数、无效数、背景内点率、重投影中位数、格数、覆盖率和reliability_ms。
- B2 metadata保存新增生效参数；运行目录保存配置快照、命令、commit及源代码/二进制归档或哈希。

分母：candidate_count=上一帧被查询的候选描述子数；matched_count=描述子及唯一对应门控后、深度检查前的对应数；valid_count=有效三维残差数；match_ratio=matched/candidate；valid_match_ratio=valid/matched。

depth_invalid_count与depth_edge_count对已描述子匹配的深度拒绝互斥计数，每条对应最多一次；match_invalid_count=candidate-matched；duplicate_match_count是其子集，不能相加。association_invalid_count为实例配对失败标志（0/1），不是特征数。numeric_invalid_count为数值拒绝记录。增强关闭时新增拒绝诊断计数不启用，零值不表示旧算法没有深度或匹配问题。

quality_sampling_ms是候选深度采样与粗边界质量采样段的计时，非纯增量成本，也不包含全部增强计算。processing_ms保留整个B2 Process耗时，包括匹配和聚合中的增强检查；track_mean_ms用于端到端对照。实例CSV重复帧级时间，统计时按frame_id去重，不按人数累加；comparison.csv中由实例CSV计算的这两项均值以存在实例行的帧为分母，端到端时间仍包含全部输入帧。

点级原因：DEPTH_MISSING、DEPTH_NEIGHBORHOOD_INSUFFICIENT、DEPTH_OUT_OF_RANGE、DEPTH_EDGE、DEPTH_QUALITY_NOT_CHECKED、DESCRIPTOR_INVALID、MATCH_INSUFFICIENT_NEIGHBORS、MATCH_HAMMING_REJECTED、MATCH_RATIO_REJECTED、MATCH_MUTUAL_REJECTED、MATCH_DUPLICATE_TARGET、GEOMETRY_INVALID、NUMERIC_INVALID、POSE_WEAK。

实例沿用INSUFFICIENT_MATCHES、LOW_MATCH_RATIO、INSUFFICIENT_VALID_DEPTH、LOW_VALID_MATCH_RATIO、INSUFFICIENT_BACKGROUND_REFERENCE、POSE_WEAK和身份原因，追加NUMERIC_INVALID、DUPLICATE_CANDIDATE_ID。具体粗阶段原因单列pose_quality_reason：INSUFFICIENT_BACKGROUND_POSE_SAMPLES、LOW_BACKGROUND_INLIER_RATIO、BACKGROUND_REPROJECTION_HIGH、BACKGROUND_COVERAGE_LOW、BACKGROUND_NUMERIC_INVALID，另保留原初始化/恢复/回退原因。

## 文件与调用位置

- `include/Deep/B2ReliabilityConfig.h`：参数和验证。
- `include/DepthNeighborhood.h`：带实例掩膜的候选深度检查及共享深度差阈值。
- `src/Deep/DynamicDetector.cc`、`include/Deep/DynamicDetector.h`：沿原解析路径读取并打印。
- `src/B2MotionEvidence.cc`、`include/B2MotionEvidence.h`：ExtractCandidates采样；ProcessInput身份与历史验证；MatchPersonFeatures去重；ComputeGeometryResiduals点级验证；ComputeInstanceDecisions实例门控；WriteCsv诊断。
- `include/CoarsePoseQuality.h`、`src/Tracking.cc`：OptimizeInitialPoseWithDiagnostics保存背景匹配分母；FinalizeCoarsePoseQuality在粗边界采样并门控；RunRGBD证据段传递快照，不引用精细结果。
- `src/FrameSemanticAnnotator.cc`：仅复用阈值计算；`src/B3SemanticGeometryFusion.cc`：追加原因码映射。
- `CMakeLists.txt`：新增针对性测试及原独立粗质量测试的OpenCV包含路径；`.gitignore`：隔离新构建目录。
- `tools/b2_reliability_test.cc`、`tools/validate_b2_reliability.py`、`configs/b2_reliability/...`：测试、运行、统一评测。
- `tools/audit_b2_reliability_logs.py`：按frame_id/track_id联结B2/B3/B4，审计证据阻断、计数不增长和重复更新；输出原因分布，不把系统标签当运动真值。

## 构建、测试、复现实验

构建最多双线程，独立输出`build_b2_reliability`。Optimizer.cc单独编译以控制峰值内存，其源码及优化等级未改。日志保留前次构建/测试失败，不覆盖旧记录。

```bash
cd /home/aaa/ORB-SLAM3_original0
python3 tools/validate_b2_reliability.py \
  --build-dir build_b2_reliability \
  --output build_b2_reliability/validation_next \
  --full-sequences
```

输出目录必须不存在。先针对性测试、旧回归、非法配置，再对sitting_static和walking_xyz各跑off/on前100帧，通过后再各跑完整序列。固定同一程序/库、原模型及其CPU推理配置，关闭全部窗口。ATE用evo平移误差、SE(3)对齐、无尺度修正、t_max_diff=0.02；RPE用相邻关联帧delta=1 frame/all_pairs，分别输出m和deg。保留txt及zip。短测不是完整精度评估。

## 剩余限制

局部max-min对单个深度离群值也可能敏感，过严会降低有效证据；光照、重复纹理和遮挡仍可能产生通过ORB门控的错误对应。背景统计并非独立真值；小区域相机运动、掩膜误检或静态背景不足可能使额外门控长期拒绝证据，进而让S过期、减少位姿/建图支撑。新增检查不是人体漏检解决方案。

必须同时看有效覆盖率、动态泄漏、静态保留、失跟、耗时和ATE/RPE。单次序列对照不能证明统计显著改善，更不能凭自身S/P/D标签认定运动真值。若ATE升高或有效证据大量下降，应保留关闭组并先核查拒绝原因，不在测试序列上随意调阈值。

## 本轮实际验证与结果（2026-11-03）

结果根目录：`/home/aaa/ORB-SLAM3_original0/build_b2_reliability/validation_02`。`comparison.csv`包含短测和完整序列；`validation_summary.json`记录执行状态，`legacy_regressions/summary.json`记录30项原有检查，`evidence_blocking_audit.json`记录B2/B3/B4联结审计及原因分布。

构建成功：主库、ablation_sequence、YOLO烟测、新增测试、相关旧测试、mono_tum和stereo_kitti；最多双线程。已有Eigen弃用等警告未在本轮修复。原Examples/RGB-D/rgbd_tum和原lib/libORB_SLAM3.so的SHA256未变化。

保留的失败记录：首轮去重代码的const指针推导错误已修复；原coarse_pose_quality_test缺少OpenCV包含路径已修复；validation_01中NaN用例失败源于测试夹具MatExpr赋值复用浅拷贝描述子内存，改为独立分配后通过。日志分别保留在build.log、build_retry.log、validation_01/；最终构建见build_resume_01.log，针对性重建见build_targeted_02.log。

针对性测试、多类别回归、原有30项回归/兼容检查、开启/关闭配置和5组非法配置均通过。两序列各两组100帧短测均通过，再运行以下4次完整序列。所有窗口关闭；同一新程序、库、模型，两个YAML仅增强Enable不同。运行期间未改代码或阈值，`artifacts.sha256`检验通过。

| 序列 | 增强 | 输入/轨迹/匹配 | ATE RMSE m | ATE median m | RPE平移RMSE m | RPE旋转RMSE deg | LOST / RECENTLY_LOST |
|---|---|---|---|---|---|---|---|
| sitting_static | off | 680/680/676 | 0.009560 | 0.007086 | 0.006378 | 0.171098 | 0/0 |
| sitting_static | on | 680/680/676 | 0.011263 | 0.009854 | 0.008128 | 0.198598 | 0/0 |
| walking_xyz | off | 827/827/826 | 0.021224 | 0.012059 | 0.014381 | 0.422152 | 0/0 |
| walking_xyz | on | 827/827/826 | 0.343006 | 0.266431 | 0.022502 | 0.584662 | 0/0 |

| 序列 | 增强 | 可推动B4的实例帧比例 | sum(valid)/sum(matched) | 平均Track ms |
|---|---|---|---|---|
| sitting_static | off | 80.81% | 97.79% | 223.99 |
| sitting_static | on | 98.32% | 82.20% | 217.93 |
| walking_xyz | off | 24.73% | 95.27% | 211.94 |
| walking_xyz | on | 23.07% | 81.55% | 218.63 |

这不是精度验收成功：walking开启组ATE明显回归，不能作为推荐默认配置，保持Enable默认0。无LOST不代表位姿准确。运行时间差异也不能当作新增模块的纯开销；它包含YOLO波动及策略反馈对SLAM工作的改变。

审计结果：4次完整运行所有P/D精细优化输入=0，最终禁止地图观测=0，关键帧索引错位=0；B4无重复frame_id/track_id行；共1842条被阻断实例帧未增加Ns/Nd/Nr，允许有效期清零/退P。sitting的MOVING=0、P_TO_D=0，两组均如此。两组身份关联决策逐行相同，不应将本次差异解释为新身份算法的收益或缺陷。

walking诊断：MOVING实例帧34→7，P_TO_D 4→0；B2的POSE_WEAK实例帧2→76，INSUFFICIENT_BACKGROUND_REFERENCE实例帧93→262；开启组粗阶段额外记录背景重投影不达标13帧、背景质量样本不足51帧。深度边缘拒绝5455条对应，其他深度无效拒绝3966条；关键帧155→241。这些是统计相关性，尚不能证明唯一根因。

合理怀疑：深度/位姿门控减少测量后改变了R/E/P分布和B4确认连续性，可能抑制真实运动证据，进而改变特征使用及地图准入；轨迹变化又影响后续背景参考，存在闭环反馈。也有SLAM线程非确定性。walking不是逐帧运动真值，不能认定被删的高残差都是正确或错误对应。

下一步建议先保持关闭组，固定验证集做多次重复，并拆分“深度质量单独开启”“粗位姿质量单独开启”的独立消融，再人工核查拒绝点与运动片段。不要在当前测试结果上直接调R/E/P或B4阈值。非person真实动态数据、非零畸变模型、长期真实动态泄漏率、ID/运动外部真值和多次重复精度未验证。

复现审计（输出JSON不得已存在）：

```bash
python3 tools/audit_b2_reliability_logs.py build_b2_reliability/validation_next
```
