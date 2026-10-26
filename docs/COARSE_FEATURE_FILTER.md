# 第9批：上一时刻 B4 状态驱动的粗位姿过滤

本批仅启用 RGB-D 正常跟踪分支的粗位姿过滤。未实现精细过滤、地图准入、YOLO 异步化或后端修改。

## 开关与配置

新增完整配置 `configs/frontend_modes/rgbd_tum3/mode3_coarse_filter_validation.yaml`。
它保留第8批 TUM3 标定、ORB、B2 六阈值与 B4 参数，只更改：

```yaml
DynamicFrontend.Mode: 3
TwoStage.CoarseFilter.Enable: 1
FeaturePolicy.EvidenceOnly: 0
```

依赖 `TwoStage.Enable=1`、`TwoStage.UseCoarsePoseForB2=1`、`TwoStage.Fallback.Enable=1`、
`FeatureSemantic.Enable=1`、`FeaturePolicy.Enable=1`，YOLO 与 B2/B3/B4 开启，B1 提取阶段过滤关闭。
`TwoStage.FinePolicy.Enable=0`、`TwoStage.MapAdmission.Enable=0` 必须保持关闭。
B2/B3/B4 仍要求 `EvidenceOnly=1`：它们本身不修改 SLAM；独立的粗位姿策略应用步骤会修改匹配输入，因此整个 Mode=3 不再是 EvidenceOnly。

旧配置不改写，Mode=0/1/2 和未指定 Mode 的 legacy 路径保留。
Mode=2 禁止启用任何实际过滤。旧 `mode3_action_reserved.yaml` 仍拒绝启动；它不满足本批明确的 coarse-only 组合，不能被静默解释成完整 ACTION。

## 调用关系

```text
GrabImageRGBD：清空本帧回退原因，整图 ORB，语义标注，冻结上一 B4 状态的策略快照
  Track：保留原有分支、初始化、Atlas 与重定位逻辑
    TrackWithMotionModel / TrackReferenceKeyFrame
      原始匹配（包括原有最小匹配数与运动模型扩大搜索窗）
      OptimizeNormalInitialPose
        CoarseFeatureFilter：备份 → 按上一状态清关联 → 原有 PoseOptimization
        质量不足：恢复 → 对完整原始匹配重新 PoseOptimization
      原有外点处理与分支成功判据
    FinalizeCoarsePoseQuality
    B2 → B3 → B4（只读、每输入帧一次）
    原始 TrackLocalMap
    原始关键帧决策与后端
```

重定位不调用新过滤包装器，原来的 `Relocalization()` 源码未修改。
未初始化、新地图、重定位/恢复上下文以及纯 VO 情况不应用过滤。
任何一次过滤回退原因在本输入帧内保持有效；即使后续参考关键帧跟踪成功，也不会重新推动本帧 B4。

## 策略与索引

允许背景与上一时刻为 S 的人物内部点；禁止 P、D、新人物默认 P、人物边界与未知实例。
只按原有索引更新 `mvpMapPoints[i]`、`mvbOutlier[i]`、策略元数据，不删除/重排关键点、描述子或改变 N。
禁止项置空关联，置 `mvbOutlier=false`，置 `mvDynamicExcluded=true`。
`mvFeaturePolicy` 与冻结的上一状态策略保持一致。

`mvDynamicExcluded` 在本批表示**粗位姿阶段策略排除**，不是优化外点，也不是“从整个 SLAM 生命周期永久删除”。
后续未经策略过滤的 TrackLocalMap 仍可能重新匹配这些位置；本批不承诺精细跟踪或建图排除人物。
这也是为什么不能把本批配置当作已完成的两阶段动态前端。

## 恢复与可靠性

事务备份：关联、外点标志、优化前位姿、策略与排除标志。
过滤后匹配数或优化内点数使用现有 YAML 的 `TwoStage.MinCoarseInliers`（默认30），
内点比例使用 `TwoStage.MinCoarseInlierRatio`（默认0.30）；无新硬编码可调阈值。
另外保留原分支需要的地图内点数10；定位模式不强制地图内点数，因为原流程允许纯 VO。
背景内点也须满足已有最小粗位姿内点阈值；背景排除区与第7/8批一致，为原始人物并集加原有整数圆盘膨胀。

过滤失败先恢复所有备份，再执行原始优化，之后仍由原分支/Track 决定参考关键帧重试、成功或丢失。
优化异常恢复输入后向上传递，不静默吞错。
本帧 B2 位姿可靠性强制置低，B3 不可靠，B4 的 Ns/Nd/Nr 冻结（不可靠次数可增加）。

回退原因包括：

- `COARSE_FILTER_INSUFFICIENT_MATCHES`
- `COARSE_FILTER_INSUFFICIENT_INLIERS`
- `COARSE_FILTER_POSE_NOT_RELIABLE`
- `COARSE_FILTER_INSUFFICIENT_MAP_INLIERS`
- `COARSE_FILTER_INSUFFICIENT_BACKGROUND_INLIERS`

## 日志口径

每条正常优化尝试输出一行 `[COARSE FILTER]`，同时写入唯一命名的 `coarse_feature_filter_*.csv`。
一帧若运动模型失败后重试参考关键帧，可能有两行，`branch` 用于区分，不能误计为两次 B4 更新。

`coarse_matches_before / coarse_policy_rejected / coarse_matches_after` 描述**本次过滤尝试**，满足 before−rejected=after。
rejected 只计算被移除的非空匹配，不是所有被标注为禁用的 ORB 点数。
`coarse_inliers` 是过滤尝试内点数；因匹配不足而未执行过滤优化时为0。
`coarse_fallback` 与 `coarse_fallback_reason` 说明恢复情况；`restored_inliers` 单独记录恢复后原始优化的内点数，未恢复为−1。
`skip_reason` 表示不满足应用上下文或本帧已回退；这类记录不是已执行过滤。

`feature_policy_summary_*.csv` 保留原有字段：Mode=3 的 evidence_only=0；
actual_excluded_count 是粗位姿策略禁用的**全部主 Frame 点数**，包括原先无匹配的点。
fine_allowed/map_allowed 仍是建议统计，不代表本批已实施精细过滤或地图准入。
B2/B3/B4 的 applied_action/policy 保持 `NONE_EVIDENCE_ONLY`，其逐帧只读检查继续有效。

## 编译与验证

使用 `build_yolo11` 隔离构建，`-j2`；共享 Tracking 头变动时先单独编译重型 Optimizer 对象，再构建其它目标。
原版 `Examples/RGB-D/rgbd_tum` 与 `lib/libORB_SLAM3.so` 用编译前后 SHA256 检查，不作为构建输出。

新增 `coarse_feature_filter_test`：上一/当前状态区分、允许与拒绝类别、排除不等于外点、
优化前与优化后回退、恢复先于回退优化、位姿恢复、地图/背景内点门控、异常恢复。
`tools/check_frontend_mode_configs.py` 保留旧配置检查并新增 coarse-only 合法/非法组合。
`tools/run_coarse_filter_fallback_test.py` 在新目录用30帧分别强制匹配数和内点比例回退，不改正式 YAML。
`tools/audit_coarse_feature_filter.py` 检查未改重定位/原始 Track/精细跟踪/关键帧分支、统计一致性、回退冻结与 once-only 次序。

数据集先 sitting/walking 各100帧，通过后才完整运行；所有配置快照、commit、命令、日志、CSV 和轨迹保存在新时间戳目录。
编译通过不等于精度或轨迹优于基线。

### 已完成的前置验收

- 独立构建完成：RGB-D、Stereo、Monocular 示例、验证 runner、YOLO smoke 与测试目标。
- 8项单元测试通过；39项合法/非法配置检查通过。
- 原版可执行程序和库的编译前后 SHA256 一致。
- 四个100帧短片段通过：Mode=2/3 × sitting_static/walking_xyz，均100帧OK、无LOST。
- Mode=3 短片段：sitting 共排除2,984个匹配、walking 共排除4,527个匹配；各99次正常优化均应用过滤，未自然触发恢复。
- 两个独立30帧强制恢复测试通过：过滤后匹配不足、过滤后内点比例不足，各29个恢复帧；
  均保留30帧OK、无LOST，原始优化真实重跑，B4有效计数冻结。
- 短片段源码/运行审计通过：重定位与原始 Track/TrackLocalMap/关键帧决策未改，B2阈值与算法未改，B4每帧每track只更新一次。

前置验收位置（仓库下相对路径）：

- `results/frontend_stage_validation/batch9_coarse_filter_build_01/`
- `results/frontend_stage_validation/config_checks_20261022_211427/`
- `results/frontend_stage_validation/batch9_coarse_filter_short_20261022_211442/`
- `results/frontend_stage_validation/batch9_forced_filter_fallback_20261022_211630/`

### 完整序列验收结果

完整结果目录：
`/home/aaa/ORB-SLAM3_original0/results/frontend_stage_validation/batch9_coarse_filter_full_20261022_211659`。
每个运行子目录包含生效配置、commit、完整命令、runtime/config SHA256、run.log、B2/B3/B4 CSV、
粗位姿质量 CSV、策略 CSV、两条轨迹、退出码及 validation.json；Mode=3 另有粗过滤 CSV。

| 模式 | 序列 | 输入/相机轨迹行数 | 关键帧轨迹行数 | 粗过滤次数 | 匹配 before→after | 排除匹配数 | 自然回退 |
|---|---|---:|---:|---:|---:|---:|---:|
| 2 | sitting_static | 680/680 | 13 | 0 | 不应用 | 0 | 不应用 |
| 2 | walking_xyz | 827/827 | 256 | 0 | 不应用 | 0 | 不应用 |
| 3 | sitting_static | 680/680 | 15 | 679 | 168411→157684 | 10727 | 0 |
| 3 | walking_xyz | 827/827 | 227 | 826 | 180116→165098 | 15018 | 0 |

全部退出码0，所有输入帧均为OK，RECENTLY_LOST=LOST=0。初始帧是建图初始化，不应用粗过滤。
Mode=3 每条完整序列的参考关键帧分支各执行一次过滤，其余过滤来自运动模型分支。
自然运行未触发恢复，不能据此声称恢复已被真实困难场景充分覆盖；前述两组受控测试验证了具体恢复实现。

`coarse_filter_audit.json` 对短/完整共8个运行全部通过：

- 直接核对第9批修改前源码，Track、Relocalization、TrackLocalMap、关键帧决策、初始化与 Atlas 控制流保持不变；
- B2算法和接口与第8批一致，优化器、匹配器、LocalMapping、地图点、ORBextractor等源码哈希未变；
- 每帧只有一次证据处理，每track只有一次B4更新，不可靠证据不改变状态和有效计数；
- B2/B3/B4仍标记 NONE_EVIDENCE_ONLY，B2前后的位姿、关联、外点和描述子检查通过。

`extraction_policy_audit.json` 另核对两模式、两个序列、短/完整片段：
逐帧 raw_fast、final_orb 数量相同，B1提取阶段 mask_rejected 均为0。
这证明本批过滤发生在使用阶段，不证明描述子逐字节相同或轨迹精度相同。

汇总见 `coarse_filter_run_summary.csv`，含所有短/完整运行、退出码、轨迹行数和匹配过滤量。
原版程序/库 SHA256 前后相同，未提交Git、未切换分支，未覆盖旧配置或旧实验结果。
修改前源码归档和本批原有文件差异位于 `batch9_coarse_filter_build_01`；新增文件包含在最终 implemented_sources.tar.gz 中。

本批未评测新模式的 ATE/RPE，未对GUI显示、真实跟踪丢失后的恢复做数据集实测，也未开启精细过滤或地图准入。
关键帧和轨迹可因粗过滤及后端异步调度变化，不应仅根据上述通过结果宣称精度或实时性能提升。

## 重复验证命令

在 `/home/aaa/ORB-SLAM3_original0` 中执行（脚本自动新建时间戳目录，不覆盖结果）：

```bash
python3 tools/run_frontend_stage_validation.py \
  --stage batch9_manual_validation \
  --mode-config 2=configs/frontend_modes/rgbd_tum3/mode2_b2_explicit_coarse_evidence.yaml \
  --mode-config 3=configs/frontend_modes/rgbd_tum3/mode3_coarse_filter_validation.yaml \
  --require-main-person
```

此命令先执行全部100帧片段，再执行完整 sitting_static 和 walking_xyz；Viewer关闭，避免图形环境阻塞。
只运行短片段加 `--short-only`；审计新目录用 `python3 tools/audit_coarse_feature_filter.py <新结果目录>`。
恢复到本批不影响SLAM的对照，直接选旧 Mode=2 配置，不要在 Mode=2 内打开过滤开关。
