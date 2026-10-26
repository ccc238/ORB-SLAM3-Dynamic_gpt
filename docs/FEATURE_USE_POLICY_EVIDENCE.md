# 第 5 批：统一 FeatureUsePolicy（仅建议与统计）

## 开关与范围

同一 DynamicDetector YAML 配置读取器新增：

```
FeaturePolicy.Enable: 1
FeaturePolicy.EvidenceOnly: 1
FeaturePolicy.SaveSummaryCsv: 1
FeaturePolicy.PrintFrameSummary: 1
```

Enable 默认 0，其他默认 1。开启要求 FeatureSemantic.Enable=1、显式 Mode=2；
EvidenceOnly=0 明确报错。旧配置默认不计算、不打开 CSV、不打印策略统计。
保存 CSV 和终端统计可以独立关闭。

完整实验配置新建为：
`configs/frontend_modes/rgbd_tum3/mode2_feature_policy_evidence.yaml`。
没有覆盖旧 YAML，没有改变相机、ORB、B2/B3 阈值或 B4 的 3/6/10/15。
TwoStage.CoarseFilter / FinePolicy / MapAdmission 等实际动作开关仍为 0；Mode 3 仍拒绝启动。

## 唯一策略实现

`FeaturePolicyEvaluator.h/.cc` 提供只接收 const Frame 的纯计算接口：

- ComputeFeaturePolicies(frame, COARSE)：读取 mvPreviousB4State。
- ComputeFeaturePolicies(frame, FINE / MAPPING)：读取 mvCurrentB4State。
- FeaturePolicyAllows：USE_NORMAL 允许所有阶段；USE_POSE_ONLY 只允许两种位姿阶段，
  不允许建图。当前表不生成 USE_POSE_ONLY，它只保留清晰的扩展语义。
- SummarizeFeaturePolicies：核查粗策略快照未被覆盖，生成各阶段统计。

| 特征 | 粗建议 | 精细/建图建议 |
|---|---|---|
| 背景 | USE_NORMAL | USE_NORMAL |
| 人体内部，阶段对应状态 S | USE_NORMAL | USE_NORMAL |
| 人体内部，阶段对应状态 P | EXCLUDE_PENDING | EXCLUDE_PENDING |
| 人体内部，阶段对应状态 D | EXCLUDE_DYNAMIC | EXCLUDE_DYNAMIC |
| 人体边界，不论 S/P/D | EXCLUDE_BOUNDARY | EXCLUDE_BOUNDARY |
| UNKNOWN 语义或无有效实例/track | EXCLUDE_UNKNOWN | EXCLUDE_UNKNOWN |

有可靠实例但阶段对应状态 UNKNOWN，也在该阶段返回 EXCLUDE_UNKNOWN。
新 track 的历史状态由上一批统一初始化为 P，因此三个阶段建议都不使用它，
直到状态机按规则累计证据后进入 S。粗和精细的时间必须分开：

```
Frame + 语义/历史状态标注
    -> ComputeFeaturePolicies(COARSE)，保存局部快照
    -> 原有完整 Track（不读取建议）
    -> B2 -> B3 -> B4，仅一次状态更新
    -> 回填当前 B4 元数据
    -> 计算 FINE / MAPPING 建议、CSV 与终端统计
```

S->P 帧：该人物内部点粗建议允许，但精细/建图建议拒绝。
P->S 帧：粗建议拒绝，精细/建图建议允许。D->P 仍不允许，新 P 也不允许。

重要：这仍是完整 Tracking 后做证据计算的旧调度，并非已经实现粗/精两阶段
Tracking。建议向量只在独立局部结果中存在，不写入 Frame::mvFeaturePolicy；
该数组仍表示“尚未应用任何新策略”，全为 USE_NORMAL，mvDynamicExcluded 全 false。
任何已应用标志会使本统计模块报错，而不是输出误导的 applied=0。
没有修改地图点指针、外点标志、位姿、匹配、优化、关键帧或建图入口。

## CSV 与计数定义

文件为 B2.Debug.OutputDir（现有实验配置为当前结果目录）下唯一时间戳命名的
`feature_policy_summary_*.csv`，每个输入帧一行。文件已存在则报错，不覆盖。

用户要求的九列完整保留：

```
coarse_allowed_count,fine_allowed_count,map_allowed_count,
background_count,S_count,P_count,D_count,boundary_count,unknown_count
```

还记录 frame_id、timestamp、N、previous_S/P/D/unknown_count、evidence_only=1、
policy_applied=0、actual_excluded_count=0。

- S/P/D_count：仅统计有效实例归属的人体**内部**点，采用当前 B4 状态。
- boundary_count：PERSON_BOUNDARY，单独计数，不再计入 S/P/D。
- unknown_count：UNKNOWN 语义、无有效归属的人体内部点，或当前状态 UNKNOWN。
- previous_*：有效归属的人体内部点的历史状态，用于复现粗建议。
- background + S + P + D + boundary + unknown = N。
- coarse_allowed = background + previous_S。
- fine_allowed = map_allowed = background + S（当前状态）。

当有效实例仅当前状态未知、历史仍为 S 时，粗建议仍按历史 S，精细/建图拒绝；
这不同于“实例归属未知”，后者三个阶段全部拒绝。真实 Mode 2 正常输出中，
B4 会为当前每个可见 person 提供状态。

所有 allowed_count 都是策略资格数量，不是实际匹配、优化内点或生成地图点数量。
现有深度、位姿等 SLAM 门控并未放宽。当前 YOLO 漏检时，未覆盖像素仍遵循
上一批 BACKGROUND 标注；本批没有新增掩膜传播、ReID 或额外运动判据。

## 验证

所有构建使用隔离 build_yolo11，最多 -j2。刷新依赖后单独编译重型对象，避免
Optimizer 与其他重型翻译单元并发消耗内存：

```
cmake -S . -B build_yolo11
gmake -C build_yolo11 -f CMakeFiles/ORB_SLAM3.dir/build.make CMakeFiles/ORB_SLAM3.dir/depend -j2
cmake --build build_yolo11 --target src/Optimizer.o -- -j2
cmake --build build_yolo11 --target ablation_sequence feature_policy_test feature_semantic_test frontend_mask_test b4_state_machine_test instance_track_lifecycle_test b2_candidate_test yolo_seg_smoke rgbd_tum stereo_euroc mono_tum -- -j2
python3 tools/run_frontend_stage_validation.py --stage batch5_feature_policy --mode-config 2=configs/frontend_modes/rgbd_tum3/mode2_feature_policy_evidence.yaml
python3 tools/audit_feature_policy.py <本次结果目录>
python3 tools/audit_feature_semantics.py <本次结果目录> --reference results/frontend_stage_validation/step5_shared_track_20261022_153908
python3 tools/audit_shared_track_updates.py <本次结果目录>
```

feature_policy_test 覆盖全部 16 种前后状态组合、S->P/P->S、新 P、边界/未知优先级、
无效 ID、计数/CSV、空帧、粗快照校验、USE_POSE_ONLY，以及不修改 Frame 元数据。
既有单元测试验证主特征、描述子、地图点、外点和位姿未被标注改变。
新增启动检查覆盖依赖错误、EvidenceOnly=0 拒绝和正常开关配置。

本批不进行精度改善宣称，也不要求多线程 SLAM 的最终轨迹逐位相同。

## 完成验收（2026-10-22）

中断前编译进程已失效，保留 build.log，随后以相同 -j2 增量完成全部目标，
成功日志为 `results/frontend_stage_validation/batch5_feature_policy_build_20261022_01/build_resume.log`。
六项单元/回归测试全部通过；20 项新旧配置检查通过，记录在
`results/frontend_stage_validation/config_checks_20261022_164417/`。
额外一帧运行证明 Enable=1 且保存/打印开关均为 0 时仍正常运行、不生成策略 CSV。

在线结果目录：
`results/frontend_stage_validation/batch5_feature_policy_20261022_164439/`。
先完成两个 100 帧测试，再完成 sitting 680 帧和 walking 827 帧；四次退出码均为 0，
所有输入帧为 Tracking OK，RECENTLY_LOST=0、LOST=0。

两个完整运行的 CSV：

- `full_mode_2_sitting_static/feature_policy_summary_1792658733567158484.csv`
- `full_mode_2_walking_xyz/feature_policy_summary_1792658884149616298.csv`

以下为累计特征观测数量（不是唯一物理点）：

| 完整序列 | 粗允许 | 精细允许 | 建图允许 | 背景 | S | P | D | 边界 | 未知 |
|---|---|---|---|---|---|---|---|---|---|
| sitting | 629320 | 629599 | 629599 | 469415 | 160184 | 4008 | 0 | 51057 | 0 |
| walking | 740874 | 741386 | 741386 | 700811 | 40575 | 36834 | 24915 | 29299 | 62 |

完整 walking 有 9 帧的粗/精细允许数不同，sitting 有 2 帧。例如 walking
frame 6 为 652 / 965 / 965（粗 / 精细 / 建图），frame 29 为 921 / 635 / 635；
两种方向均出现，说明没有用当前状态回写粗策略快照。

`feature_policy_audit.json` 核查全部九列和计数公式；
`feature_semantic_audit.json` 核查主特征数组、实际策略全为 NORMAL、实际剔除为 0，
且逐帧 FAST/ORB 数量与关闭模块的原参考完全一致；
`shared_track_audit.json` 核查 ID、冻结规则与每帧每 track 只更新一次。
所有审计通过。实际应用标记均为 0，B4 状态没有被策略计算额外推进。

主要修改位置为 FeaturePolicyEvaluator.h/.cc（唯一策略规则与 CSV），Tracking.h/.cc
（输出流和前后两个诊断调用点），FeatureSemanticTypes.h / DynamicFrontendConfig.h /
DynamicDetector.cc（同一配置体系），CMakeLists.txt（测试目标），以及新测试和审计脚本。
没有修改 ORBmatcher、Optimizer、LocalMapping 或地图点准入，没有覆盖旧配置/结果，
没有自动提交 Git。真正的粗/精 Tracking 拆分与策略应用仍未实现。
