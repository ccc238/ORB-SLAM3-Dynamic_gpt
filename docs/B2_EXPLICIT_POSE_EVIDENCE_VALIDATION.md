# 第8批：B2显式位姿接口与同阈值EvidenceOnly对照

## 实现范围

B2仍使用独立person候选ORB，不把它加入主Frame；YOLO保持同步。
不启用粗/精特征过滤、地图准入、优化边权重或后端修改，不调任何B2阈值。
保留旧配置和结果，不自动Git提交。所有编译最大-j2，输出仍在build_yolo11隔离目录。

新增显式接口：

```cpp
B2Result Process(
    const B2CandidateFrame* previous,
    const B2CandidateFrame& current,
    const Sophus::SE3f& currentFromPrevious,
    bool currentPoseReliable);
```

previous=nullptr表示没有历史，不能产生可靠运动证据。
当前帧与上一帧都只包含B2候选、值类型背景观测与位姿快照，没有Frame/MapPoint/Map指针。
Process先复制输入，再处理；previous可以指向B2自身的PreviousFrame，复制发生在替换历史前。
旧Process(current)保留为兼容适配器，旧单元测试与接口调用继续可用。

Tracking显式准备：
`T_current_from_previous = current.Tcw * previous.Tcw.inverse()`。
前后历史必须同为coarse或同为final，poseSource标签不一致时报错。
B2的person残差和背景残差均只使用传入的相对位姿，不在几何函数内重建另一套变换。
新增单元测试：绝对Tcw均为单位阵，但传入0.10m平移，person和背景残差均必须为0.10m；
同时验证弱位姿不能产生可靠结果、混合来源拒绝、bootstrap正确保存当前历史。

当前粗位姿可靠且上一帧可靠才计算几何残差；弱位姿本批选择不计算无意义残差，
用-1表示不可用，不填0。所有弱位姿/无历史实例明确标记GEOM_UNRELIABLE、POSE_WEAK。
当前可靠但没有上一帧时仍保存当前可靠快照，避免后续一直无法建立有效帧对。
粗位姿质量门控、回退与B4冻结沿用第7批；不以最终位姿补算粗证据。
新显式路径bootstrap从旧的NOT_EVALUATED统一为POSE_WEAK，仅影响诊断标签，
旧Process(current)兼容接口保留其旧bootstrap行为，不改变SLAM。

## 精确R/E/P公式与当前阈值

本批配置基于第7批实际生效的
`configs/frontend_modes/rgbd_tum3/mode2_coarse_quality_evidence.yaml`，
不是按模型名称或另一份旧YAML猜测阈值。仍由现有DynamicDetector统一解析。

对有效匹配，米制深度反投影得到P_prev、P_curr：

```
r_i = ||P_curr_i - T_current_from_previous * P_prev_i||_2   [m]
R   = median(r_i)
```

背景参考使用person原始掩膜及原有膨胀边界外的Tracking内点匹配：

```
m_bg     = median(r_background)
sigma_bg = max(1.4826 * median(|r_background - m_bg|), 0.01 m)
e_i      = max(0, r_i - m_bg(depth_bin_i)) / sigma_bg(depth_bin_i)
E        = median(e_i)
P        = count(e_i >= 3.5) / valid_geometry_count
```

深度分桶0.50m；每桶至少30个样本，否则使用本帧全背景参考；
全部背景少于50个样本则几何不可靠。两帧的掩膜排除、深度有效性、匹配及位姿门控
仍然先执行；只有全部通过才进入三区域决策。r2d只记录，不用于运动硬判定。

| 决策 | R [m] | E | P |
|---|---|---|---|
| MOVING，三条件同时满足 | >=0.020 | >=0.40 | >=0.10 |
| STATIC，三条件同时满足 | <=0.018 | <=0.10 | <=0.06 |
| UNCERTAIN | 上述两类之间 | | |
| UNRELIABLE | 任一可靠性门控失败 | | |

特别注意：3.5是点级e_i高残差计数阈值，来自B2.Geometry.Decision.MovingThreshold；
0.40是实例级E运动阈值，来自B2.Geometry.NormMovingThreshold，不能互换。
旧Decision.MinMovingFraction=0.60等兼容字段仍在完整YAML中保留，
但本配置的六个显式阈值优先生效，P运动门控是0.10，不是0.60。
本批不替换公式、不改变背景MAD算法，也不将旧定义的阈值搬到新定义上。

## 两份完整配置与调用时序

- `configs/frontend_modes/rgbd_tum3/mode2_b2_explicit_coarse_evidence.yaml`
- `configs/frontend_modes/rgbd_tum3/mode2_b2_explicit_final_reference.yaml`

两者所有B2字段完全相同，唯一区别是TwoStage.UseCoarsePoseForB2的1/0；
其余相机、ORB、B3/B4、模块开关相同。
正式目标是coarse：初始跟踪汇合 -> 质量门控 -> B2/B3/B4 -> TrackLocalMap -> 关键帧。
final仅作受控比较：完整Track返回后计算B2/B3/B4，不是上线目标路径。
所有组B2/B3/B4.EvidenceOnly=1，实际动作开关全0。

[B2 INPUT]记录来源、上一帧ID、当前/帧对可靠性和显式变换标志；
[B2 READONLY]记录主Frame不可变验证与NONE_EVIDENCE_ONLY。
在B2/B3/B4前后核查主Frame的mvpMapPoints、mvbOutlier、N、描述子和位姿，
有任何改变立即报错。位姿用参数字节对比，避免未改变的NaN导致错误比较。
B2关闭时跳过这些克隆与检查。元数据、日志和Viewer不参与算法。

无Map/Atlas写入接口被传给B2，后端、匹配器、优化器、地图点等源码另做哈希核查。
LocalMapping正常异步更新不等于B2修改地图；不能要求两个独立多线程运行的地图
或最终轨迹逐位一致。本批不关闭后端、不为得到相同地图而改动SLAM调度策略。

## 验证与输出

先完成两种来源各自的sitting/walking前100帧（共四次），全部通过后才完整运行四次。
结果全部写入新目录，保存配置、commit、完整命令、日志、退出码、轨迹、CSV、运行库哈希。
保持原来的800/600余帧关联顺序，不把walking所有实例帧当作运动真值。

```bash
python3 tools/run_frontend_stage_validation.py --stage batch8_explicit_b2 --mode-config final=configs/frontend_modes/rgbd_tum3/mode2_b2_explicit_final_reference.yaml --mode-config 2=configs/frontend_modes/rgbd_tum3/mode2_b2_explicit_coarse_evidence.yaml --require-main-person
python3 tools/compare_b2_pose_stages.py <本批结果目录>
```

comparison目录输出：

- b2_metric_quantiles_comparison.csv：R/E/P分布，count、P05/P25/P50/P75/P90/P95/P99、均值/标准差/最大值。
- b2_background_reference_comparison.csv：背景样本量、背景median及sigma分布，辅助解释归一化变化。
- b2_point_residual_quantiles_comparison.csv：另列单个候选匹配的raw/normalized残差分布，
  不与实例级R/E混用，不据此套用实例阈值。
- b2_category_comparison.csv：四种GEOM状态、POSE_WEAK实例帧数。
- b2_unreliable_reason_comparison.csv：不可靠原因分解，用于区分阈值问题和背景/匹配覆盖问题。
- b4_transition_comparison.csv：P_TO_S、P_TO_D、S_TO_P、D_TO_P及可见S/P/D实例帧数。
- paired_instance_metrics.csv：按frame_id/timestamp/instance_id关联两组R/E/P、状态及差值。
- comparison_metadata.json：实际YAML全部B2字段、有效float32阈值、精确公式、队列时序和只读审计。

分别报告“指标存在”“可靠性门控通过”和“两组共同通过”的统计口径。
UNRELIABLE/-1不作为零误差，不视为静态，不混入有效分布。
GEOM_UNCERTAIN表示B2可靠性门控通过但三区域判定不确定；不等于B3可靠融合。
状态与转移计数单位为实例帧/事件，不是图像帧数；同帧多个人分别统计。

独立两次运行会混入正常后端线程调度差异，配对统计不是严格的单一因果实验。
粗/精阶段可用的背景内点集合也不同；这不是固定全部观测、只替换T的纯位姿消融。
任何阈值调整都需另开验证实验并经确认；本批没有自动调参分支。

## 完成验收（2026-10-22）

中断后原编译会话已失效，保留build.log，使用相同-j2增量完成编译，
成功日志为：
`results/frontend_stage_validation/batch8_explicit_b2_build_20261022_01/build_resume.log`。
RGB-D、Stereo、Mono兼容目标及7项单元/回归测试全部通过；
28项配置检查通过，记录在
`results/frontend_stage_validation/config_checks_20261022_203017/`。
新增显式0.10m相对变换、弱位姿、混合来源拒绝和bootstrap历史测试均通过。
复用了第7批强制回退工具，新测试输出位于
`results/frontend_stage_validation/batch7_forced_fallback_20261022_203021/`
（目录前缀沿用工具名称，但运行的是本批运行库）：
20帧原始跟踪成功，19次普通局部地图跟踪完成，B4全P且Ns/Nd/Nr全0。

正式对照结果：
`results/frontend_stage_validation/batch8_explicit_b2_20261022_203400/`。
四次100帧检查全部通过后才开始完整序列；8次均退出码0、全部Tracking OK，
RECENTLY_LOST=0、LOST=0。所有3414个输入帧的只读检查通过。

| 组别 | 序列 | 100帧相机/关键帧轨迹行数 | 完整相机/关键帧轨迹行数 |
|---|---|---|---|
| final参考 | sitting_static | 100 / 7 | 680 / 12 |
| coarse目标 | sitting_static | 100 / 7 | 680 / 14 |
| final参考 | walking_xyz | 100 / 47 | 827 / 306 |
| coarse目标 | walking_xyz | 100 / 53 | 827 / 292 |

两组运行库SHA256相同：
`a2afc492f5d316ab81f54722e8ed1ee770017aa72c8e2f52b12483f484496ed6`。
模型SHA256相同：
`205e029a8be9f835cf798bb56cd95e8927b56b359b37dbe400bfee777adb808d`。
YAML除UseCoarsePoseForB2外全部字段相同，B2全部字段也与第7批实际配置相同。
HEAD仍为34c523ee399af8c1ef82721b5fbf64f64618a76e，未自动Git提交。

### 完整序列状态计数（实例帧）

| 序列 | 来源 | STATIC | MOVING | UNCERTAIN | UNRELIABLE |
|---|---|---|---|---|---|
| sitting | final | 1111 | 0 | 247 | 2 |
| sitting | coarse | 1110 | 0 | 246 | 4 |
| walking | final | 239 | 33 | 319 | 429 |
| walking | coarse | 212 | 24 | 254 | 530 |

### 完整序列B4转移计数

| 序列 | 来源 | P_TO_S | P_TO_D | S_TO_P | D_TO_P | D实例帧 |
|---|---|---|---|---|---|---|
| sitting | final | 2 | 0 | 0 | 0 | 0 |
| sitting | coarse | 2 | 0 | 0 | 0 | 0 |
| walking | final | 6 | 4 | 4 | 3 | 342 |
| walking | coarse | 6 | 3 | 3 | 3 | 290 |

### 完整序列共同有效样本上的分布

按同一frame_id/timestamp/instance_id配对，且两组均通过B2可靠性门控：
sitting共同1356个实例帧，walking共同479个实例帧。
以下各格为P50 / P95，不把无效-1计作0。

| 序列 | 来源 | R [m] | E | P |
|---|---|---|---|---|
| sitting | final | 0.007757 / 0.012234 | 0 / 0 | 0.032345 / 0.090350 |
| sitting | coarse | 0.007678 / 0.012169 | 0 / 0 | 0.032026 / 0.090909 |
| walking | final | 0.015424 / 0.036349 | 0 / 1.237098 | 0.036585 / 0.147168 |
| walking | coarse | 0.015839 / 0.037273 | 0 / 1.229300 | 0.037383 / 0.147168 |

共同有效样本分布接近，但不能因此忽略覆盖下降：
walking最终位姿通过B2可靠性门控591/1020，粗位姿为490/1020；
粗位姿UNRELIABLE增加101个实例帧，主要是背景参考不足由120增至219。
背景样本数P50由80降至58；背景sigma P50约0.017287/0.017410m，变化较小。
POSE_WEAK由2增至8；粗位姿质量本轮记录首帧初始化及2帧背景支持不足回退，
并按前一帧无效的规则传播至相应帧对，不推动B4。

### 审计与结论

comparison_metadata.json确认公式、当前有效阈值、配置/模型/运行库一致；
所有已通过可靠性门控的实例均按原六阈值重新计算，分类一致。
coarse目标的B2输入与证据结束均发生在TrackLocalMap前；
final仅是完整Track后计算的比较参照，不作为目标前端。
两组B3 applied_action、B4 applied_policy全部NONE_EVIDENCE_ONLY。
主Frame的地图点关联、外点、N、描述子和位姿未被证据链改变。
共享ID、每帧每track单次更新、UNRELIABLE冻结与策略未应用审计均通过。
逐帧ORB提取数量两组一致，且与第7批目标组一致。
后端/MapPoint/优化/匹配器源码哈希不变，B2没有地图写入接口。

结论：未观察到必须重调R/E/P阈值的整体失效：sitting未出现MOVING/D，
walking保留MOVING和P_TO_D，配对分布接近。保持现有阈值，未进行任何调参。
下一步应优先抽查粗阶段背景参考不足的帧及其覆盖率，而非降低运动阈值凑证据。
不能把UNRELIABLE或UNCERTAIN算作静态/运动，也不能把walking所有帧当运动真值。

限制：未做人工运动真值标注、GUI抽查或ATE/RPE精度评测；
没有人为触发真实重定位/切图分支。不同关键帧数量可受原有异步后端调度影响，
不能据此声称B2改写地图、精度改善或两次轨迹完全一致。
本批止于EvidenceOnly验收，不自动进入特征过滤/地图准入阶段。
