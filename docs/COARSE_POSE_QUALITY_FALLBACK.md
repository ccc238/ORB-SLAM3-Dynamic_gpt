# 第7批：粗位姿质量与原始跟踪回退

## 范围与配置

新增完整配置，不覆盖以前的 YAML：
`configs/frontend_modes/rgbd_tum3/mode2_coarse_quality_evidence.yaml`。
仍为 Mode=2、UseCoarsePoseForB2=1，B2/B3/B4 EvidenceOnly=1，
CoarseFilter/FinePolicy/MapAdmission 全为0；B2阈值与B4的3/6/10/15不变。

```yaml
TwoStage.MinCoarseInliers: 30
TwoStage.MinCoarseInlierRatio: 0.30
TwoStage.Fallback.Enable: 1
```

在现有 DynamicDetector YAML 解析器读取，两项缺省即30/0.30；
内点数必须为正整数，比例必须在(0,1]且有限。
这些只是调试起点，不是最终论文阈值。旧的 final-pose/legacy/Mode0/Mode1 路径
不执行新质量门控；开启粗位姿的新旧配置都会使用这里的默认值或显式值。
粗位姿模式当前要求 Fallback.Enable=1；设为0明确报错，不能绕过安全门控。
Mode3及三个实际动作开关仍拒绝启动。继续只支持非惯性RGB-D。

## 质量定义

`include/CoarsePoseQuality.h` 定义用户要求的 CoarsePoseQuality、
独立的 CoarsePoseContext 与纯函数 EvaluateCoarsePoseQuality。

- match_count：被选中的原有初始位姿优化调用**之前**非空MapPoint关联数量。
- inlier_count：同一次原有 Optimizer::PoseOptimization 的返回值。
- inlier_ratio：inlier_count/match_count，分母0时为0。
- median_reprojection_error：同次优化后有效内点的二维欧氏重投影误差中位数，
  单位像素，不按尺度层加权；无有效误差时为-1。本批只记录，不设新的误差阈值。
- just_relocalized：本帧重定位尝试成功且 mnLastRelocFrameId 对应当前输入帧。
- reliable/reason：质量门控结果及明确原因。

原流程会清除优化外点的指针，因此不能用分界点剩余指针反推原匹配分母，
否则会虚增内点率。OptimizeInitialPoseWithDiagnostics 包装**同一次**优化调用，
不增加优化次数、不更改返回值。分别用于参考关键帧、运动模型和重定位分支。
TrackLocalMap 的优化调用完全不改。仅定位模式如果保留MM解、重定位失败，
同时恢复MM解对应的质量快照；重定位尝试标志仍保留，本帧按重定位回退。
未开启粗位姿时包装器直接执行原优化调用，不额外遍历特征。

背景候选和背景内点：使用主Frame原始mvKeys像素，排除当前所有raw person
实例mask及B2现有backgroundExclusionDilateRadius边界。膨胀核采用精确整数圆盘，
与B2的dx²+dy²<=r²查询一致；不使用B1掩膜，不改变ORB/Frame/MapPoint。
背景候选数、背景内点数均复用MinCoarseInliers作为下限，暂不添加独立阈值。
背景内点只统计非空关联且非外点；B2原有深度/背景残差门控还会进一步检查。

## 原因优先级与回退

按以下优先级，第一项失败原因写入fallback_reason：

1. NEW_MAP：新建/切换Map、正常建图模式LOST将创建地图、地图重置后初始化。
2. NOT_INITIALIZED：第一次未初始化输入。
3. RELOCALIZATION：本帧进入过原有Relocalization函数（包括成功）。
4. JUST_RECOVERED：从RECENTLY_LOST/LOST恢复OK或成功重定位后的下一输入帧。
5. COARSE_TRACK_FAILED：原有初始跟踪bOK=false。
6. INSUFFICIENT_COARSE_FEATURES：背景候选数不足。
7. INSUFFICIENT_COARSE_INLIERS：总内点数不足。
8. INSUFFICIENT_COARSE_FEATURES：背景内点数不足。
9. POSE_NOT_RELIABLE：无有限有效位姿、纯VO、计数异常或内点率不足。

全部通过为COARSE_POSE_RELIABLE，fallback_reason为空；阈值边界采用>=。
NEW_MAP优先于NOT_INITIALIZED，重定位发生帧优先于恢复冷却帧。
保留mbCreatedMap/Atlas原有生命周期，不为诊断改写或重置原有SLAM状态。

回退继续原始分支所决定的TrackLocalMap/失败处理/关键帧逻辑，不修改bOK或mState。
本帧B2 poseValid=false，不使用最终精细位姿补算粗证据。
B2保留诊断但不给可靠三维证据；B3不能输出可靠融合，B4按UNRELIABLE冻结。
RunRGBDGeometryEvidence新增断言，回退时如果B3意外产生可靠结果立即报错。
Ns/Nd/Nr、S/P/D状态冻结；不可靠计数和原有TTL/年龄仍正常推进。
没有证据不等于静态；不以无证据恢复S。独立B2历史保存无效位姿，
因此下一帧即使粗质量通过，也可能因上一位姿无效而继续被B2门控。

记录输入frame_id/timestamp不再依赖Atlas早退后可能被重置的mCurrentFrame，
避免回退证据错误使用默认Frame身份。B4仍每输入每track只更新一次。
首帧初始化的原有初始关键帧创建例外继续保留。

## 输出与复现

开启粗位姿时在已有B2.Debug.OutputDir创建唯一的
`coarse_pose_quality_<timestamp>.csv`；已存在报错，不覆盖。
每帧一行保存全部质量字段、背景数、上下文标志、实际阈值、
fallback及fallback_reason；每行flush，写入失败报错。
终端每帧一行[COARSE QUALITY]。原B2/B3/B4 CSV和策略CSV继续保存。

构建仍使用隔离build_yolo11，全部命令最大-j2，重型Optimizer对象先独立编译。
新增coarse_pose_quality_test覆盖所有原因、阈值等号、无效计数与纯函数不改输入。
run_coarse_quality_fallback_test.py单独运行20帧，使用100000的测试阈值强制回退：
检查原始局部地图跟踪仍完成、轨迹非空、B3全不可靠、B4全部P且Ns/Nd/Nr全0。
该配置仅为测试输出快照，绝不覆盖正式配置。

```bash
python3 tools/run_coarse_quality_fallback_test.py
python3 tools/run_frontend_stage_validation.py --stage batch7_coarse_quality --mode-config 2=configs/frontend_modes/rgbd_tum3/mode2_coarse_quality_evidence.yaml --require-main-person
python3 tools/audit_coarse_pose_quality.py <本批目录>
python3 tools/audit_shared_track_updates.py <本批目录>
python3 tools/audit_feature_policy.py <本批目录>
python3 tools/audit_feature_semantics.py <本批目录> --reference results/frontend_stage_validation/batch6_coarse_pose_20261022_171118
```

先通过合成质量测试和强制回退测试，再执行两个100帧测试，随后完整sitting/walking。
审计源码确认原有初始分支控制流、优化调用次序/次数不变，
在线逐帧重放质量规则并核查fallback的B2/B3/B4、时序、计数和实际策略未应用。
本批不进行轨迹精度提升宣称，不自动Git提交。

## 完成验收（2026-10-22）

全部目标以-j2编译通过，Optimizer对象先独立构建；
主库、RGB-D、Stereo、Mono及7项单元/回归测试均完成。
第一次构建在CMake运行中再生成目标后，旧Make进程没有识别新增测试目标，
报No rule to make target coarse_pose_quality_test；保留build.log，
再次增量构建成功，最终日志build_finalize.log，非C++编译错误。
构建日志：`results/frontend_stage_validation/batch7_coarse_quality_build_20261022_01/`。
26项配置检查通过：
`results/frontend_stage_validation/config_checks_20261022_173236/`。

强制回退实测：
`results/frontend_stage_validation/batch7_forced_fallback_20261022_173239/`。
20帧全部回退，全部Tracking OK，退出码0；相机轨迹20行、关键帧轨迹5行。
19个非初始化帧仍完成原有TrackLocalMap，B3全部不可靠；
B4一直为P，Ns/Nd/Nr始终为0，策略实际应用和剔除均为0。
这是独立测试配置（MinCoarseInliers=100000），不作为正式实验结果或默认参数。

正式配置在线结果：
`results/frontend_stage_validation/batch7_coarse_quality_20261022_173255/`。
两个100帧测试与短片段质量审计先通过，然后完整运行两个序列。

| 测试 | 帧数 / OK | RECENTLY_LOST / LOST | 相机 / 关键帧轨迹行数 | 质量通过 / 回退 | 退出码 |
|---|---|---|---|---|---|
| sitting 100帧 | 100 / 100 | 0 / 0 | 100 / 8 | 99 / 1 | 0 |
| walking 100帧 | 100 / 100 | 0 / 0 | 100 / 41 | 99 / 1 | 0 |
| sitting 完整 | 680 / 680 | 0 / 0 | 680 / 16 | 679 / 1 | 0 |
| walking 完整 | 827 / 827 | 0 / 0 | 827 / 253 | 826 / 1 | 0 |

全部正式运行仅首帧以NOT_INITIALIZED回退；低质量、重定位等情况没有在这些自然
运行中触发。七项合成/单元测试覆盖所有8个原因和B4不可靠冻结，强制实测验证
INSUFFICIENT_COARSE_FEATURES的真实回退链路。没有强制触发真实新地图、重定位、
JUST_RECOVERED流程；这些只核查源码标志接入与合成质量规则，不能宣称已全面实测。

完整质量CSV：

- `full_mode_2_sitting_static/coarse_pose_quality_1792661628720264885.csv`
- `full_mode_2_walking_xyz/coarse_pose_quality_1792661778917463289.csv`

四项审计全部通过：

- coarse_quality_audit.json：逐帧重算质量/回退原因，B2/B3回退不可靠、B4状态冻结，
  时序正确；原初始分支控制流及原优化次序/调用点不变，后端等源码哈希不变。
- shared_track_audit.json：共享ID、单次更新、Ns/Nd/Nr冻结和TTL规则通过。
- feature_policy_audit.json：许可计数分区正确，实际策略应用/剔除数均为0。
- feature_semantic_audit.json：数组与N对齐，逐帧raw_fast、final_orb及B1剔除数
  与第6批相同，人体主ORB保留，未新增跟踪过滤。

粗位姿质量通过并不等于B2几何可靠：B2仍有背景样本数、深度与匹配门控。
完整walking的B3 MOVING=26、B4 D=298；sitting二者为0，均为实例帧数量，
不是视觉运动真值或精度指标。多线程调度会影响地图与最终轨迹，
本批未计算ATE/RPE，未做GUI人工抽查，不宣称与原版轨迹逐位一致。

HEAD保持34c523ee399af8c1ef82721b5fbf64f64618a76e，分支master1，
未自动Git提交，原有修改与旧配置/旧结果保留。
结果保存生效配置、commit、命令、CSV、轨迹、日志、退出码、运行库哈希，
以及本批实现源文件归档和工作区补丁。
后续真正动作模块应以CoarsePoseQuality.reliable为执行前提；
当前动作开关仍拒绝启用，本批不会自动进入下一阶段。
