# 第6批：粗位姿分界与证据调用前移

本批仅移动 RGB-D 的 B2/B3/B4 调用位置，不启用特征策略、地图准入、优化加权、
YOLO 异步化或后端修改。不修改 B2 残差算法、B3 规则、B4 的 3/6/10/15 参数。
不自动提交 Git；以前的配置与结果保留。

## 配置

新增完整 TUM3 配置（不覆盖第5批配置）：

`configs/frontend_modes/rgbd_tum3/mode2_coarse_pose_evidence.yaml`

```yaml
DynamicFrontend.Mode: 2
TwoStage.Enable: 1
TwoStage.CoarseFilter.Enable: 0
TwoStage.UseCoarsePoseForB2: 1
TwoStage.FinePolicy.Enable: 0
TwoStage.MapAdmission.Enable: 0
B2.EvidenceOnly: 1
B3.EvidenceOnly: 1
B4.EvidenceOnly: 1
```

UseCoarsePoseForB2 缺省仍为 0；旧配置保持完整 Track 后计算证据的调度。
Mode=3 与三个实际动作子开关仍明确拒绝启动，不会悄悄应用策略。
本批粗位姿调度仅支持非惯性 RGB-D；Stereo/IMU_RGBD 请求它会明确报错。
FeatureSemantic / FeaturePolicy 在新增配置中开启，仅做属性标注与许可统计。

## 调用位置与原有分支

保留 Tracking::Track() 在统一分界点前的全部原有控制流：
运动模型/参考关键帧选择及回退、RECENTLY_LOST、重定位、仅定位模式、
初始化、时间戳与 Atlas 逻辑。未改成固定顺序的简化跟踪器。

```
GrabImageRGBD
  同步 YOLO -> 统一实例关联 -> B2 独立候选提取
  -> 主 Frame 整图 ORB -> 语义/历史状态标注 -> 保存粗策略建议
  -> Track
       原有初始跟踪各分支
       -> 统一分界点
       -> 粗位姿可靠性检查
       -> RunRGBDGeometryEvidence
            B2 -> B3 -> B4（一次）
            -> 更新当前状态元数据 -> 精细/建图策略建议与统计
       -> 原有 TrackLocalMap（不读取策略建议）
       -> 原有 Tracking 状态/运动模型维护
       -> NeedNewKeyFrame -> CreateNewKeyFrame -> LocalMapping 队列
  -> 若 Track 早退且尚未处理：不可靠证据收尾一次
  -> 日志与 Viewer -> 返回最终位姿
```

## 粗位姿可靠性与失败语义

可靠要求原有初始跟踪 bOK=true、HasPose=true、SE3 矩阵全部有限；
仅定位的纯 VO 分支 mbVO=true 不作为可靠地图位姿证据。
未额外改变 SLAM 的匹配数或位姿优化成功阈值。B2 原有匹配率、深度有效性、
背景参考数量等可靠性门控继续生效。

B2 同时复制粗阶段 Tracking 内点的 MapPoint ID、像素和米制深度，用于背景参考；
只把值交给 B2，不把 MapPoint 指针、Frame 引用交给它。B2 内部历史保存的位姿
也来自同一个阶段，不能上一帧精细位姿配当前帧粗位姿混用。

粗跟踪失败或 Track 提前返回时，B2 poseValid=false；B3/B4 只获得不可靠证据，
状态与有效计数冻结。不会以精细阶段位姿重新计算本帧证据，也不会重复更新 B4。
首次无历史 B2 帧本来就没有跨帧几何证据。

## 关键帧顺序与必要初始化例外

普通跟踪帧：保持 NeedNewKeyFrame 在 TrackLocalMap 返回之后；
CreateNewKeyFrame 新增守卫，粗位姿模式下如果本帧证据或 TrackLocalMap 尚未完成，
不允许普通关键帧插入。LocalMapping::InsertKeyFrame(pKF) 前另有可审计日志。
保留原有失败分支细节：粗跟踪失败时 TrackLocalMap 不会被强行调用；
RECENTLY_LOST 分支可能仍查询 NeedNewKeyFrame，但普通 RGB-D 失败帧不满足原有
插入条件，且新增入队守卫禁止在未完成精细跟踪时创建普通关键帧。

**首帧初始化例外不可去掉**：StereoInitialization 必须先创建初始 Map/KeyFrame，
后续才有局部地图可跟踪。本批按要求保持此逻辑，只增加
bootstrap_keyframe / INITIAL_MAP_REQUIRED 事件；不会宣称初始化也执行过精细跟踪。
初始化帧以无可靠粗位姿收尾，不推动 B4 状态转换。

## 一次性与 EvidenceOnly 约束

每个 RGB-D 输入重置 ready/processed/fineCompleted；Frame 完成后开启本帧事务。
统一方法先检查 ready 且未 processed，随后标记已处理；重复调用直接报错。
Track 返回后仅在尚未处理时收尾，processed=true 时绝不重复 B2/B3/B4。
原有 B4 每 track 每帧重复更新检测保留。

Frame::mvFeaturePolicy 仍全 USE_NORMAL，mvDynamicExcluded 全 false；
策略建议只计算，不传给 ORBmatcher、Optimizer、TrackLocalMap 或建图。
关键帧和地图点仍用原有算法。提前运行证据与 CSV 会影响时间/线程调度，
因此不承诺多线程最终轨迹逐位一致，也不把本批运行当作精度改善证明。

## 验证和复现

构建只用隔离 build_yolo11，最大 -j2，Optimizer 对象单独构建避免内存峰值。
先运行两个100帧测试，再运行完整 sitting_static 与 walking_xyz；任何验收失败停止。
通用运行器保存完整配置、commit、命令、日志、B2/B3/B4 CSV、轨迹、退出码。

```bash
python3 tools/run_frontend_stage_validation.py --stage batch6_coarse_pose --mode-config 2=configs/frontend_modes/rgbd_tum3/mode2_coarse_pose_evidence.yaml --require-main-person
python3 tools/audit_coarse_pose_stage.py <本批结果目录>
python3 tools/audit_shared_track_updates.py <本批结果目录>
python3 tools/audit_feature_policy.py <本批结果目录>
python3 tools/audit_feature_semantics.py <本批结果目录> --reference results/frontend_stage_validation/batch5_feature_policy_20261022_164439
```

新增 TWO STAGE 日志记录 coarse_boundary -> evidence_complete -> fine_complete
-> keyframe_decision -> keyframe_enqueue -> frame_complete。
初始化单独标识，无效粗位姿不会冒充有效证据。
audit_coarse_pose_stage 对比第5批保存的源码，核查原始分支未改写、
下游算法未改、普通入队严格晚于精细跟踪、每帧仅处理一次证据。
共享 ID/计数冻结、特征属性与许可分区分别由已有三个审计工具检查。

## 完成验收（2026-10-22）

本批全部目标以 -j2 编译通过，保留在：
`results/frontend_stage_validation/batch6_coarse_pose_build_20261022_01/`。
Optimizer 单独构建后再编译主目标；最后再次增量构建确认无遗漏源文件。
六项单元测试通过，包含新增的当前/上一帧粗位姿无效 -> POSE_WEAK 检查；
21项新旧配置检查通过，目录为
`results/frontend_stage_validation/config_checks_20261022_171108/`。

B2 测试第一次因测试工作目录缺少它要求的 build_yolo11/validation 子目录而退出，
没有进入算法测试。失败日志保留在 unit/b2_candidate_test.log；
补齐独立测试目录后通过，日志为 unit/b2_retry01/b2_candidate_test.log。
没有为此修改 B2 算法或输出目录合法性检查。

在线结果新目录：
`results/frontend_stage_validation/batch6_coarse_pose_20261022_171118/`。
各次均保留 effective_config.yaml、commit.txt、command.sh、run.log、exit_code.txt、
validation.json、B2/B3/B4/策略 CSV、两条轨迹及计时统计。

| 测试 | 帧数 / OK | RECENTLY_LOST / LOST | 相机轨迹行数 | 关键帧轨迹行数 | 退出码 |
|---|---|---|---|---|---|
| sitting 100帧 | 100 / 100 | 0 / 0 | 100 | 7 | 0 |
| walking 100帧 | 100 / 100 | 0 / 0 | 100 | 38 | 0 |
| sitting 完整 | 680 / 680 | 0 / 0 | 680 | 15 | 0 |
| walking 完整 | 827 / 827 | 0 / 0 | 827 | 252 | 0 |

全部通过：

- coarse_pose_short_audit.json / coarse_pose_stage_audit.json：统一分界前的分支
  与第5批快照完全一致；7个跟踪/Atlas函数原文不变；关键帧普通入队严格晚于精细跟踪。
- shared_track_audit.json：共享ID一致，最大每帧每track更新次数为1，
  不可靠证据冻结有效计数与状态。
- feature_policy_audit.json：九项许可/分类计数正确，所有实际应用/剔除标志为0。
- feature_semantic_audit.json：数组对齐，逐帧 raw_fast、final_orb、b1_mask_rejected
  与第5批参考一致，B1剔除数为0，人体主ORB正常保留。

完整 sitting：B3 MOVING=0、B4 D=0；
完整 walking：B3 MOVING=25、B4 D=302。这里全部是**实例帧计数**，不是运动真值。
粗位姿阶段背景参考与残差分布不同于旧的精细位姿阶段，不能把计数差异直接解释为
精度优劣；本批未调阈值，也未实施特征使用或地图准入。
关键帧轨迹记录的是最终保留关键帧，不等于所有入队关键帧数量（后端可正常剔除）。

HEAD 仍为 34c523ee399af8c1ef82721b5fbf64f64618a76e，分支 master1，
未自动提交，已有用户改动保留。保存第5批到第6批的精确 Tracking 补丁
tracking_changes_from_batch5.patch、工作区补丁与本批源文件归档以复现未提交实现。

未验证内容：未强制制造真实重定位/LOST/Atlas切图运行分支（只核查其源码未被改写）；
位姿失效降级由合成 B2 单元测试覆盖。本批在线测试关闭 GUI，未人工检查窗口；
未执行 ATE/RPE 或视觉运动真值标注，不宣称轨迹精度改善或逐位一致。
新粗位姿模式暂不支持 Stereo/IMU_RGBD；共享目标只做编译兼容检查。
Mode 3、实际粗/精过滤与地图准入仍未启用，等待后续批次。
