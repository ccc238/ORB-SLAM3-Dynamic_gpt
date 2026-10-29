# 实例身份关联改进：实现与复现

本轮基于 `snapshot_clean` / `47ba0c174edfc4cfd7b6399b0f5b80c6b5b3b4b9`，保留已有 B4 有效期默认开启改动。不自动提交、切换分支或推送。下列阈值是工程调试初值，不是精度验证结论。

## 文件清单

| 文件 | 本轮内容 |
|---|---|
| include/Deep/InstanceAssociation.h（新） | 关联结果、状态、统一配置与参数校验 |
| include/Deep/InstanceTrackManager.h、src/Deep/InstanceTrackManager.cc | 旧贪心对照、预过期、LK/速度预测、门控、匈牙利、歧义、生命周期事件 |
| include/YOLOSeg.h | 实例携带关联结果；不改变网络输出解码 |
| include/Deep/DynamicDetector.h、src/Deep/DynamicDetector.cc | 同一 YAML 解析模块读取新参数、打印生效阶段 |
| include/B2MotionEvidence.h、src/B2MotionEvidence.cc | 复制关联权限、严格相邻测量门控、清理历史接口与身份失败原因 |
| src/B3SemanticGeometryFusion.cc | 新的身份不可靠原因码映射，原融合分支不变 |
| include/B4TemporalStateMachine.h、src/B4TemporalStateMachine.cc | 单轨迹证据重置、终止同步、A3 不可信证据拦截 |
| include/Tracking.h、src/Tracking.cc | 实际灰度/米制深度接入、粗阶段历史许可、统一重置、关联 CSV 和优化器输入计数入口 |
| src/TrackingDiagnostics.cc、src/FrontendDiagnosticLog.cc | 关联权限与实际优化器输入计数列，状态/动作诊断保持分离 |
| tools/b2_candidate_test.cc | 显式声明独立合成匹配所需的几何配对许可 |
| tools/instance_association_test.cc（新） | 预测、分配、歧义、TTL、无效输入与 A0～A3 测试 |
| tools/identity_state_integration_test.cc（新） | 不可信旧 S、B2 测量错配、B4 冻结、粗/精细/KeyFrame 准入与模块重置 |
| tools/identity_system_reset_test.cc（新） | 实际 System 重置、重启时间线，供外部脚本审计 sequence_id 与 ID 不复用 |
| tools/validate_instance_association.py（新） | 单元/配置/短测/完整序列和不覆盖输出的审计驱动 |
| tools/evaluate_instance_identity.py（新） | 独立人工标注模板及身份/运动评测接口 |
| tools/test_instance_identity_evaluation.py（新） | 合成人工标注验证指标、恢复时间、缺真值拒绝与防覆盖，不作为真实身份精度 |
| tools/summarize_instance_association_runs.py（新） | 运行审计汇总、系统轨迹连续性、源文件哈希与未提交 diff 快照 |
| CMakeLists.txt | 新增三个测试目标，沿用已有独立 bin/lib 输出隔离 |
| configs/instance_association/rgbd_tum3/A*.yaml（新） | 四份完整且互不覆盖的消融配置 |

本文件为新增说明。开始任务时已存在的 B4Config、B4 有效期说明/测试、生命周期测试及验证脚本改动保留，不将它们重新归因为本轮身份关联算法。

## 接口及实际时序

`Tracking::GrabImageRGBD()`：

1. 校验采集时间与 Frame ID；拒绝重复、倒退、NaN，调用 System 重置后才能开始新序列。
2. 同步 YOLO；按 Camera.RGB 获得灰度，将深度按原尺度转换为米。
3. `InstanceTrackManager::Associate(instances,size,frameId,timestamp,gray,depth)`。
4. 同步 TERMINATED 事件至 B4 RemoveTrack 和 B2 ForgetTrack；重关联清理证据。
5. B2 独立候选提取、主 Frame 构造、语义属性标注。仅 allowStateInheritance 时查询 B4 GetUsableState；否则 P。
6. 保留原 Tracking 分支，粗位姿后运行 B2/B3/B4，之后执行精细策略、TrackLocalMap、关键帧及地图准入。

新增 `InstanceAssociationResult` 放在 `include/Deep/InstanceAssociation.h`，随 `YOLOSegInstance` 及 `B2PersonInstance` 传递。它保存身份、状态、代价、双向优势、上一测量帧/时间、继承/几何权限及原因。

| 结果 | ID | A3 历史状态 | 几何配对 |
|---|---|---|---|
| NEW_TRACK | 新增 | P，无历史 | 禁止 |
| CONTINUOUS_CONFIRMED | 保留 | 允许，但仍受证据有效期约束 | 仅与严格相邻且测量帧/时间一致的历史 |
| REACQUIRED | 保留 | ResetTrackEvidence → P，Ns/Nd/Nr=0，无可靠证据时间 | 返回首帧禁止；该帧作为新测量起点 |
| AMBIGUOUS | 新临时 ID | P，不继承旧对象 | 禁止 |

`B2MotionEvidence::ProcessInput()` 还检查 manager 的 previousMatchedFrame/timestamp 与 B2 自己的 previous measurement 相符。否则输出 IDENTITY_HISTORY_MISMATCH，不能仅按 ID 配对。无几何许可输出 IDENTITY_ASSOCIATION_UNTRUSTED。B2 残差公式、归一化阈值、B3 融合规则不变，仅增加身份可靠性门控/原因映射。

`B4TemporalStateMachine::Process(...,associations)` 在 A3 中再次检查权限，即使上游误传可靠 B3，也不能推动计数。不允许继承时设 P；不允许几何时冻结。ResetTrackEvidence 不推进帧、年龄或 TTL。状态仍每帧每轨迹更新一次。

## 身份关联算法

输入必须是同分辨率 CV_8UC1 二值 0/255 掩膜、合法框、类别及置信度；灰度和米制深度必须尺寸匹配。增强模式在分配前删除过期轨迹。

设最后匹配帧 L，当前 F，TrackTTL=T：

- 构建候选前，`F-L-1>T` 直接终止，不能通过匹配清零年龄复活。
- 若本帧未匹配，`F-L>T` 终止。
- 因此恰好漏检 T 帧后返回可以重关联；漏检 T+1 帧不可以。测试显式覆盖 T=2 与旧生命周期 T=15。
- ID 在 manager 生命周期内单调递增；Reset 清空历史、递增 sequence_id，但不回收 ID。
- 身份 TTL 与 B4 EvidenceExpiry 独立；后者仍使用采集时间严格大于截止时间才过期。

A2/A3 用历史原始掩膜腐蚀安全内区提角点，LK 前向/反向检查，取有效位移中位数平移框和掩膜。不依赖 SLAM 最终位姿；LK 不作为运动证据。支持不足用中心速度预测，并收紧中心门限至 0.75 倍；最多预测 0.75 秒，位移不超过目标框对角线乘中心门限。当前实现采用平移，不拟合可选仿射，也没有新增 ReID 网络或外观描述子。

共同候选门控：类别、TTL、预测中心距离（目标框对角线归一化）、框面积尺度比、最小 mask/box 重叠。深度从原始实例内稀疏采样有效米制数据（0.3～5m），足够样本且 MAD 合理才可用；缺深度不强制判不同人。深度一致性采用 `max(MaxDepthDifference, MaxDepthRelative*oldDepth)`。

代价为加权的 `1-maskIoU`、`1-boxIoU`、归一化中心距离、`1-flowSupport/MaxCorners`、深度差/容许差。缺失评分项删除后重新归一化；至少两个非中心几何证据项且总代价不超过 MaxCost。mask/box 两项并非独立统计证据，这一初版最低证据约束不能保证识别外观相似的对象。

匈牙利矩阵维度为历史轨迹数+当前实例数，包含每条历史的未匹配列、每个实例的新建行和零代价 dummy-dummy；真实匹配只有低于 MaxCost 才可分配。MaxCost 必须小于两个未匹配代价之和。不存在为了填满配额强制匹配的操作。

A2/A3 检查轨迹行与实例列的次佳真实候选优势；没有次佳时记缺失值而不是伪造数值，同时检查与未匹配总成本的优势。已分配边若不是局部优势边会被拒绝，不重试降门限。当前掩膜覆盖两个以上预测旧掩膜的大部分时拒绝继承。旧对象模板不更新，只增加丢失年龄。

临时歧义轨迹不作为后续可靠模板候选；持续合并期间可能每帧产生临时 ID。这是保守行为的明确局限，会增加碎片，必须与连续性、静态重用一起统计，不能以错误继承减少单独宣称成功。

## 配置及消融

完整 TUM3 相机、ORB 和 B2/B3/B4 参数配置：

- `configs/instance_association/rgbd_tum3/A0_legacy.yaml`
- `configs/instance_association/rgbd_tum3/A1_global.yaml`
- `configs/instance_association/rgbd_tum3/A2_predict_ambiguity.yaml`
- `configs/instance_association/rgbd_tum3/A3_controlled.yaml`

四组只改变 `InstanceAssociation.Mode`，统一关闭图片保存和逐特征 CSV，保留摘要、诊断与关联 CSV。B4 有效期四组均开启，B2/B3/B4 阈值及相机标定相同，旧配置未覆盖。

| 模式 | 过期预过滤/全局分配/共同门控 | LK/速度预测/歧义拒绝 | 重关联 B4 清零与禁止继承 |
|---|---|---|---|
| A0=0 | 否，旧贪心与旧后置 TTL | 否 | 否 |
| A1=1 | 是 | 否 | 否 |
| A2=2 | 是 | 是 | 否 |
| A3=3 | 是 | 是 | 是 |

缺失 InstanceAssociation.Mode 时默认 A3。需要复现旧身份算法必须显式填 0。A0 是旧关联器对照，不是原版 ORB-SLAM3 或逐位一致快照：公共输入校验、B2 相邻测量保护、日志仍生效。A1/A2 故意保留重关联状态继承作为不安全消融对照，不建议正式使用。单参数 C++ 构造仅兼容旧的 size-only 合成测试，显式采用 A0；生产 Tracking 总是传入 YAML 配置。

所有新增配置（四份完整示例逐项显式列出）：

```yaml
InstanceAssociation.Mode: 3
InstanceAssociation.MaxCost: 0.65
InstanceAssociation.UnmatchedTrackCost: 0.40
InstanceAssociation.UnmatchedInstanceCost: 0.40
InstanceAssociation.MinMargin: 0.08
InstanceAssociation.MaxCenterDistance: 1.0
InstanceAssociation.MaxScaleRatio: 4.0
InstanceAssociation.MinMaskIoU: 0.05
InstanceAssociation.MinBoxIoU: 0.10
InstanceAssociation.MaxPredictionSeconds: 0.75
InstanceAssociation.MaxDepthDifference: 0.60
InstanceAssociation.MaxDepthRelative: 0.30
InstanceAssociation.MaxDepthMAD: 0.20
InstanceAssociation.Weight.Mask: 0.40
InstanceAssociation.Weight.Box: 0.20
InstanceAssociation.Weight.Center: 0.25
InstanceAssociation.Weight.Flow: 0.15
InstanceAssociation.Weight.Depth: 0.15
InstanceAssociation.MergeCoverage: 0.65
InstanceAssociation.Flow.FBError: 1.5
InstanceAssociation.Flow.Quality: 0.01
InstanceAssociation.Flow.MaxCorners: 80
InstanceAssociation.Flow.MinSupport: 8
InstanceAssociation.MinEvidenceTerms: 2
InstanceAssociation.SafeErodeRadius: 3
InstanceAssociation.MinDepthSamples: 20
InstanceAssociation.Debug.SaveCsv: 1
InstanceAssociation.Debug.PrintSummary: 1
```

范围、非有限数、不合理代价、非法开关启动时报错。B0/Yolo 关闭不创建推理会话；身份模块收到空列表不引入模型依赖。Stereo/mono 算法不接入新 RGB-D 关联流程。

## 重置与下游策略

Tracking 的 Reset、ResetActiveMap、CreateMapInAtlas 通过统一 helper 清理 manager、B2、B4、当前结果、待处理证据、粗姿态历史记录。若在 Track 内创建地图，延迟到下一次 RGB-D 输入开始执行统一清理，当前输入只退休一次，避免破坏仍在处理的 owned snapshot。终止事件单独写日志。

P 仍禁止正常粗位姿授权、精细优化及新地图观测。没有删除关键点/描述子行、没有删除历史地图点、没有修改 MapPoint::AddObservation 或 BA。

重要：原有粗跟踪安全回退会恢复完整匹配再优化，这可能使用 P/边界特征。新权限不能伪装成“所有路径绝对零使用”；诊断把 RESTORED_FALLBACK 单独记录，新增入口计数也包含这类调用。精细策略仍禁止 P/D，地图准入不放开。若后续希望连应急粗回退也禁用这些特征，应另设独立实验，不在本轮悄悄改变原回退策略。

## 构建与验证命令

使用独立目录，最多双线程，原有 build 和原可执行文件不覆盖：

```bash
cd /home/aaa/ORB-SLAM3_original0
cmake -S . -B build_instance_association -DCMAKE_BUILD_TYPE=Release
cmake --build build_instance_association -j2
python3 tools/validate_instance_association.py --output build_instance_association/validation_new --short-sequences --full-sequences
```

低内存 GCC 环境可先独立构建 Optimizer.cc.o（仅编译期回收参数，不改 BA）：

```bash
make -C build_instance_association -f CMakeFiles/ORB_SLAM3.dir/build.make CMakeFiles/ORB_SLAM3.dir/src/Optimizer.cc.o -j2 'CXX_FLAGS= -Wall -O3 -O3 -DNDEBUG -march=native -fPIC -std=c++14 --param=ggc-min-expand=10 --param=ggc-min-heapsize=32768'
cmake --build build_instance_association -j2
```

验证脚本先执行 15 个单元/集成测试、配置检查，再对 A0～A3 的 sitting_static / walking_xyz 各 100 帧，全部通过后可跑 A3 完整序列。强制新输出目录；保存完整命令、实际 YAML、配置哈希、commit、退出码、耗时、全部算法日志、两条轨迹；不覆盖旧结果。图形关闭，测试不依赖窗口。每个新结果目录的 summary.json/sequence_summary.json 是实际结果而非预期值。

手动运行（末尾 0=完整序列，1=Viewer，1=按输入时间节拍；改序列/配置时使用新的目录）：

```bash
ROOT=/home/aaa/ORB-SLAM3_original0
RUN=$(mktemp -d "$ROOT/build_instance_association/manual_A3_walking_XXXXXX")
cd "$RUN"
export LD_LIBRARY_PATH="$ROOT/build_instance_association/lib:$ROOT/Thirdparty/DBoW2/lib:$ROOT/Thirdparty/g2o/lib:$ROOT/third_party_ort/onnxruntime-linux-x64-1.17.3/lib:${LD_LIBRARY_PATH:-}"
"$ROOT/build_instance_association/bin/ablation_sequence" rgbd "$ROOT/Vocabulary/ORBvoc.txt" "$ROOT/configs/instance_association/rgbd_tum3/A3_controlled.yaml" /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz /home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt 0 1 1 2>&1 | tee run.log
```

## 日志与独立标注评测

`instance_association_*.csv` 包含逐实例代价、双向 margin、LK 支持、缺深度标记、两个权限、重置原因、测量起点、关联耗时；EVENT 行记录 NEW/AMBIGUOUS/REACQUIRED/TERMINATED/SEQUENCE_RESET。没有次佳/历史时为空，不填假零。

`dynamic_frontend.csv` 的 INSTANCE 行加入相同权限与原因，还包含每次实际优化器入口遍历得到的 coarse/fine_optimizer_feature_uses 和 coarse/fine_S_feature_uses。它们计数实际非空关联输入（含重试/回退，不是不同特征的唯一数量），不等价于最终内点，也不是单纯策略建议。P 在 coarse fallback 实际使用不会误计为 S 输入。

输入计数覆盖正常初始位姿分支及 TrackLocalMap，不覆盖未修改的 Relocalization 内部优化。含重定位的评测不能把这些列当作全程序全部优化调用次数。本轮完整序列没有 LOST/RECENTLY_LOST。

准备人工标注与评测：

```bash
python3 tools/evaluate_instance_identity.py --diagnostics /绝对路径/某次运行/dynamic_frontend.csv --output build_instance_association/annotation_template_01
# 人工填写 template 中 gt_person_id、motion_label、visible、association_correct，保留系统 frame/track 对应
python3 tools/evaluate_instance_identity.py --diagnostics /绝对路径/某次运行/dynamic_frontend.csv --annotations /绝对路径/人工标注.csv --output build_instance_association/human_evaluation_01
```

独立人工 gt_person_id 在不同系统 track 中保持同一人；motion_label=STATIC/MOVING/AMBIGUOUS/OCCLUDED，不得用 B2/B4 输出代填。工具输出 ID 切换、标注观察的碎片、错误 S 继承、错误 S 实际 coarse/fine 输入次数、重关联到 S 的时间以及静态/动态特征使用比。数据稀疏或漏标会影响碎片统计；检测完全漏人需要额外的全图人体真值标注，本模板不能测 detector recall。特征使用比的分母是可见人体内部 ORB 数，分子是优化器输入次数，重试可能使比例超过 1，不能伪称概率或完整身份基准。

建议先标注两人交叉、合并/分裂、遮挡重现、旧人离开新人进入，再对四组使用完全相同的人体与运动真值评测。同时比较静态特征重用、轨迹连续性、跟踪成功率和 SE(3) ATE/RPE。没有人工真值不能宣称减少 ID switch，更不能由编译/短测或权限审计推断定位精度提升。

自动审计结果及代码快照（不需要人体真值，明确不算身份精度）：

```bash
python3 tools/summarize_instance_association_runs.py --validation build_instance_association/validation_03 --output build_instance_association/measured_audit_01
python3 tools/test_instance_identity_evaluation.py
```
