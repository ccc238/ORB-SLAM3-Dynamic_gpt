# 第12批：关键帧决策顺序与最终位姿/内点溯源

## 范围与修复原因

第11批已经将普通关键帧决策放在 TrackLocalMap 后，且 mnMatchesInliers 在局部精细优化后清零并重新统计。
发现的缺口是：粗过滤曾回退但精细跟踪后来成功时，旧 FinePolicyAllowsKeyFrame 只检查精细优化与失败标记，
未明确禁止本帧创建关键帧。本批补齐安全门控，而非重写关键帧插入策略。

不改 B2/B3/B4 算法和阈值，不改 PoseOptimization/BA/LocalMapping、地图点准入策略、单目初始化。
不新增 YAML 字段。沿用 `mode3_map_admission_validation.yaml`；其原文件和其它旧配置不覆盖。

## 修改文件

| 文件 | 修改 |
|---|---|
| `include/KeyFrameDecisionGuard.h` | 新增纯上下文/判定器，给出明确拒绝原因 |
| `include/Tracking.h` | 每帧最终快照、日志流、门控及原始判据辅助函数 |
| `src/Tracking.cc` | reset；语义帧号；精细结果快照；Need/Create 两入口门控；阶段日志与引导帧标记 |
| `tools/keyframe_decision_guard_test.cc` | 粗回退、精细失败、语义失效、支撑不足、错误数据来源等合成测试 |
| `CMakeLists.txt` | 独立纯门控测试目标，继续隔离构建 |
| `tools/audit_keyframe_order.py` | 原算法源码冻结检查、实际顺序/内点来源/禁止插入/B4与地图审计 |
| `tools/run_frontend_stage_validation.py` | 增加 kf3 验证标签，每次运行完成即审计，失败停止 |
| 本文、`docs/DYNAMIC_FRONTEND_STAGES.md` | 实现、复现、验收与限制说明 |

## 实际时序

```
原有运动模型/参考帧/重定位分支
→ 统一粗位姿质量检查
→ B2 → B3 → B4（单次）
→ 当前策略精细过滤
→ TrackLocalMap：搜索、二次过滤、优化、最终内点
→ 保存 final_frame_id / final_inliers / final_pose / 原有 required_static_support
→ NeedNewKeyFrame 门控
→ 原有关键帧判据（输入显式 final_inliers）
→ CreateNewKeyFrame 再次门控并验证本帧 Need 已批准
→ 准入快照、构造 KeyFrame、RGB-D 近点、插入 LocalMapping
```

保留原来关键帧判据的阈值、近点需求计数、参考帧/局部建图忙闲/时间间隔等策略。
`EvaluateOriginalKeyFrameNeed(int decisionInliers)` 为原函数主体，除了参数名称外逐语句相同；
源码审计通过冻结版本比较确认。普通安全门控只能否决插入，不能强制插入。

原版/旧模式调用辅助函数时传原有 mnMatchesInliers；fine action 模式只传本帧最终精细快照。
不因粗匹配很多而放宽静态支撑。支撑阈值使用现有 FineStaticSupportRequired 结果（正常30、
近期重定位50、RECENTLY_LOST恢复11，保留原优先级），不另设一套阈值。

## 拒绝条件

- COARSE_POSE_FALLBACK / COARSE_POSE_UNRELIABLE
- SEMANTIC_POLICY_UNAVAILABLE（标注/策略不齐、当前策略不一致或语义帧号过期）
- EVIDENCE_NOT_COMPLETE / FINE_TRACKING_NOT_COMPLETED / FINE_TRACKING_FAILED
- FINAL_SNAPSHOT_STALE / FINAL_POSE_INVALID / FINAL_POSE_CHANGED
- INSUFFICIENT_STATIC_SUPPORT / FINAL_INLIERS_CHANGED
- NEED_KEYFRAME_NOT_APPROVED（Create 入口另检查）

语义策略可用性重用 VerifyFineOptimizerInputs。合法的“未检测到人”与语义失效不同：
同步推理成功、当帧标注及排除策略一致时，全背景帧仍有效。模型/解码错误仍在原入口明确报错，
不静默转成全背景。位姿一致性检查使用同一帧最终矩阵、1e-6 相对容差，仅作溯源检查，不是新优化阈值。

旧 fine-only 消融（MapAdmission=0）保留其原有未过滤建点策略，成功建点后新关联不属于先前的精细优化输入。
Need/Create 批准前始终验证精细输入；创建成功后的日志重验语义/策略数组，但不将旧模式新建点误记为语义失效。
开启 MapAdmission 时，创建后仍完整验证关联。该处理不回填用于关键帧决策的最终内点数。

每帧 reset final/semantic/Need 状态，禁止沿用上一帧批准和内点。证据阶段内 B4 的计数/状态更新不变。
门控在 Need 的原有 InterruptBA 等副作用前执行，在 Create 的 SetNotStop、new KeyFrame、new MapPoint 前执行。

## 初始化例外必须单独理解

StereoInitialization 同时用于 RGB-D，它必须先建立第一张地图才能进行下一帧局部地图跟踪。
其首关键帧不可能遵循“已有地图的精细跟踪→关键帧决策”时序，故保留原算法引导路径，仍受第11批地图准入限制。
日志单独记 `bootstrap=1, reason=BOOTSTRAP_INITIAL_MAP_REQUIRED`，不计为普通插入。
这是初始化路径的显式区分，不是 Need/Create 门控的旁路开关。新地图重新初始化具有相同边界。

## 日志、审计与运行

启用现有 FinePolicy 的模式自动保存唯一 `keyframe_stage_<timestamp>.csv`，没有新增开关。
记录 phase、frame/semantic/final帧号、粗匹配/内点、最终内点、实际决策内点、支撑阈值、
粗回退、证据完成、精细成功、位姿/语义有效性、批准/创建标记和拒绝原因。
phase 包括 final_snapshot、need_decision/need_guard、create_guard、created、frame_summary、bootstrap。
终端输出 `[KEYFRAME STAGE]` 为阶段汇总，不逐特征输出。

自动验收示例（真实路径，结果目录自动唯一，不覆盖旧结果）：

```bash
cd /home/aaa/ORB-SLAM3_original0
python3 tools/run_frontend_stage_validation.py \
  --stage batch12_keyframe_order --require-main-person \
  --mode-config kf3=configs/frontend_modes/rgbd_tum3/mode3_map_admission_validation.yaml
```

脚本先两序列各100帧，全部通过后再运行 sitting_static 680帧、walking_xyz 827帧。
Viewer=0、非交互环境，保存生效配置、commit、完整命令、CSV、日志与两条轨迹。
每次运行立刻对照精细 CSV 验证 `decision_inliers == final_inliers`，对照阶段顺序和 Frontend 插帧标记，
并审计实际地图禁止观测、B4 不重复更新和不可靠冻结。

测试不等同于精度验收：真实运行若未出现回退、语义失效、跟踪失败，则相关拒绝分支仅由合成测试覆盖，
不得声称经历了真实故障恢复。ATE/RPE、人工运动标注、复杂回环和重定位仍需专项验证。

## 本机验收记录（2026-10-23）

- commit `34c523ee399af8c1ef82721b5fbf64f64618a76e`，分支 master1，未提交 Git。
  原有用户修改保留；原版 rgbd_tum 和 lib/libORB_SLAM3.so 的 SHA256 前后相同。
- 冻结/构建/测试目录：`results/frontend_stage_validation/batch12_keyframe_order_build_01`。
  保留 pre_batch12_sources.tar.gz、第11批修改版程序/库备份、各次构建日志、相对冻结版补丁和单元日志。
  编译会话中断后按增量续编，最终构建成功，没有源码补丁绕过编译错误。
- 构建始终不超过 -j2，Optimizer 对象先单独构建；RGB-D、stereo_euroc、mono_tum 和测试目标编译成功。
- 最终构建11项单元测试全部通过；门控测试覆盖四类禁入、旧快照、位姿/内点错源、Need未批准和边界值。
  bootstrap 日志标记不能绕过普通门控也通过测试。
- 48项 YAML 合法/非法组合检查全部通过：
  `results/frontend_stage_validation/config_checks_20261023_103233`。
- 最终库 SHA256：`0855d9305c8c22647d471944e9d6c8c49f54ab3339570f258d690f16b63ded86`。
  先通过新准入及旧 fine-only 两模式、两序列各100帧，共4次短测：
  `results/frontend_stage_validation/batch12_final_keyframe_short_20261023_104115`。
  旧 fine-only 关闭准入，不声称其地图无 P/D；额外检查其建点后阶段日志无语义失效误报。
- 最终完整序列：`results/frontend_stage_validation/batch12_final_keyframe_full_20261023_104321`。
  交付以该目录为准；保留更早的 short/full 目录作为开发过程证据，不覆盖或删除。

| 序列 | 输入/OK | 相机/关键帧轨迹行数 | 普通插入累计 | 初始化 | 决策次数 | 粗/精内点不同 | 非法插入/错源 |
|---|---:|---:|---:|---:|---:|---:|---:|
| sitting_static | 680/680 | 680/7 | 6 | 1 | 679 | 679 | 0/0 |
| walking_xyz | 827/827 | 827/161 | 264 | 1 | 826 | 826 | 0/0 |

两次 exit=0、RECENTLY_LOST=0、LOST=0，最终位姿溯源错误=0，地图禁止观测/数组错位=0。
关键帧“插入累计”包含后来被原有冗余剔除机制移除的关键帧，不等于轨迹最终保留行数。
本批没有修改关键帧剔除策略。

实际日志示例：walking frame9 `coarse_inliers=118 final_inliers=147 decision_inliers=147 required_static_support=50`；
sitting frame3 `139 / 171 / 171`。所有1505次普通决策都与精细CSV及Frontend pose_inliers独立核对一致。

`keyframe_order_summary.csv` / `.json` 给出顺序审计、源码冻结和每次计数；
每个子目录另有 `keyframe_order_audit.json`、`keyframe_stage_*.csv`、配置快照、命令、commit、
B2/B3/B4及精细CSV、完整日志和轨迹。B4单次更新和不可靠冻结审计通过。
源码审计确认 B2/B3/B4、TrackLocalMap、运动模型/参考帧/重定位、地图准入、BA及原关键帧判据未改变。

这两次完整运行没有自然出现 coarse fallback/fine failure，相关拒绝条件只由合成测试覆盖。
未做GUI、ATE/RPE、故障恢复、长时回环和Atlas续建精度验收。

旧 Mode0/Mode1/Mode2/旧 B4-A 各在两序列运行100帧，共8次兼容性短测全部通过：
`results/frontend_stage_validation/batch12_legacy_compatibility_20261023_104914`。
均100/100帧 OK、exit=0、LOST=0，且未生成本批 keyframe_stage CSV。
旧 fine-only 两序列短测另由最终短测目录单独审计通过。
这表示所测旧路径仍可运行，不承诺修改版与独立原版完全等速或逐帧轨迹相同。

本批结论：普通关键帧顺序、最终内点来源、安全门控及地图准入回归验收通过。
后续仍需自然/受控故障恢复与感知漏检验证；本批不把“模型成功返回空 person 列表”当成感知真值。
未来若接入异步分割，必须传递真实推理源帧号/时间戳，不能继续只依赖本批同步调用后的标注帧号。
