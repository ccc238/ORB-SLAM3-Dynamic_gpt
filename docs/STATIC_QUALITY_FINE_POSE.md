# 静态质量加权精细位姿优化

## 范围与基线

本模块只作用于纯视觉 RGB-D、完整 Mode 3、CoarseFilter/FinePolicy/MapAdmission
均开启时的最终视觉位姿优化。旧 YAML 缺省关闭。它不是新的动态分类器，不修改
B2 残差/阈值、B3 融合、B4 状态/有效期、粗定位和回退、硬过滤、关键帧判据、
地图准入、历史观测或后端 BA。没有接入 B4.UseContinuousQuality。

核查发现旧阶段文档存在“地图准入尚未实现”等历史说明；当前源码已实现该模块，
本次以实际 Tracking / FineFeaturePolicy / MapAdmission 调用为准，未改写历史文档。
当前分支和提交仅标识基础版本；未提交工作区用实验 source_diff.patch、源码哈希复现。
没有自动提交、推送、切换分支或覆盖原版二进制。

## 配置

### 修改文件与入口

| 文件 | 本次内容 |
|---|---|
| `include/Deep/StaticQualityConfig.h` | 独立配置、范围校验、米制深度权重公式 |
| `include/Deep/DynamicDetector.h`, `src/Deep/DynamicDetector.cc` | 在既有解析器读取/打印配置，校验完整Mode3依赖 |
| `include/Frame.h`, `src/Frame.cc` | N长度权重/有效标志/原因、reset与复制 |
| `include/StaticQuality.h`, `src/StaticQuality.cc` | PrepareStaticQuality只读现有几何/策略，输出固定权重及统计 |
| `include/Tracking.h`, `src/Tracking.cc` | 粗位姿快照、第二次硬过滤后准备权重、显式精优化调用、逐帧CSV与输入元数据快照 |
| `include/Optimizer.h`, `src/Optimizer.cc` | 独立加权入口；2D/3D整边加权、Huber缩放、四轮raw chi2判定 |
| `CMakeLists.txt`, `.gitignore` | 新源/测试目标，忽略隔离构建产物；不改变原版输出位置 |
| `tools/static_quality_test.cc` | 实际g2o位姿优化集成及生命周期/公式测试 |
| `tools/validate_static_quality.py` | 回归、重置、100帧及完整三次重复、evo固定口径评测 |
| `tools/summarize_static_quality.py` | 完整重复审计、聚合统计及逐帧权重摘要分位数 |
| `configs/static_quality/rgbd_tum3/quality_off.yaml`, `quality_on.yaml` | 两份完整相机配置，仅Enable不同 |
| `docs/STATIC_QUALITY_FINE_POSE.md` | 实现、复现、失败记录及结果 |

仅列本次修改；工作区原有B2可靠性、多类别等未提交改动保留，没有被当作本次新实现。

完整 TUM Freiburg3 相机配置：

- `configs/static_quality/rgbd_tum3/quality_off.yaml`
- `configs/static_quality/rgbd_tum3/quality_on.yaml`

两文件只在 StaticQuality.Enable 不同，其余相机、ORB、B2/B3/B4、关联与准入相同。
从现有 B2 reliability **关闭**配置复制；本次不顺带开启此前出现精度回归的增强门控。
模型绝对路径已检查存在，但其他机器必须自行改成真实路径，不能混用不同标定。

| 字段 | 缺省值 | 意义 |
|---|---:|---|
| StaticQuality.Enable | 0 | 独立启用开关 |
| StaticQuality.WeightMin | 0.10 | 权重下界，0 < w_min <= 1 |
| StaticQuality.DepthSigmaA | 0.01 | A，单位 m，必须 > 0 |
| StaticQuality.DepthSigmaB | 0.01 | B，单位 1/m，必须 >= 0 |
| StaticQuality.SaveSummaryCsv | 0 | 开启模块时保存逐帧统计 |

布尔必须整数0/1，数值必须有限。A>0且B>=0使所有正米制深度上sigma>0，
不限于 B2 候选深度区间。以上只是未标定实验初值，不是最优参数。
配置由 DynamicDetector 的同一 YAML 解析器读取、校验、打印。
非完整 Mode 3 组合启动报错；Tracking 构造时拒绝 Mono/Stereo/IMU 开启本功能。
旧配置 Enable=0 可不提供模型（仍需其自身关闭 YOLO 等依赖模块），不会创建本模块CSV。

## 调用顺序与公式

```text
原始初始跟踪分支/粗位姿优化（原入口）
→ FinalizeCoarsePoseQuality
→ 保存本帧 Tcw_coarse 与 frame_id
→ B2 → B3 → B4
→ ApplyFineFeaturePolicy(pre_local_map)
→ TrackLocalMap
    UpdateLocalMap → SearchLocalPoints
    → ApplyFineFeaturePolicy(pre_pose_optimization)
    → VerifyFineOptimizerInputs
    → PrepareStaticQuality（只遍历现存非空关联）
    → Optimizer::PoseOptimizationStaticQuality
    → 原最终地图内点计数/成功条件
→ 原关键帧及地图准入
```

取主 Frame.mvDepth[i]：Frame::ComputeStereoFromRGBD 已在原关键点坐标取过深度，
输入深度由 GrabImageRGBD 转为米。本模块不重新采样，不混用去畸变像素，不读取B2候选数组。

`Xc = Tcw_coarse * Xw; z_pred=Xc.z; r_z=abs(z_meas-z_pred)`

`sigma(z_meas)=A+B*z_meas^2; u=r_z/sigma; w=max(w_min,1/(1+u^2))`

这是本工程的迁移设计，不声称逐式复现 SGDO-SLAM。粗位姿和权重在精优化之前冻结，
不在优化迭代内更新。无可靠粗位姿、粗阶段发生回退、无当前帧快照、无效深度、
预测非有限或在相机后方时取中性1。中性1不计入有效深度质量样本。
不修改主Frame的N、关键点/描述子、地图关联或策略；禁止项绝不恢复。

Frame 保存 N 长度的权重、有效标志和原因。所有图像构造器在已知 N 后通过
ResetSemanticMetadata 初始化它们；复制构造显式复制，编译器赋值逐成员复制；
默认空Frame有空数组。ResetStaticQuality恢复1/false/NOT_EVALUATED。
不修改 KeyFrame/Atlas 序列化。关闭时无 Prepared 状态或新CSV。
后续历史稳定性可作为独立质量计算器，扩展 PrepareStaticQuality 的质量上下文和有效性记录，
最终仍只产出同索引的固定标量权重；当前没有伪造历史分数或提前实现该因子。

## 优化、Huber 与外点

原 `Optimizer::PoseOptimization(Frame*)` 保留独立旧入口，内部 weighted=false。
显式加权入口要求已准备、数组对齐和精细策略校验通过；调用者仅为RGB-D TrackLocalMap。
二维 EdgeSE3ProjectXYZOnlyPose 和 RGB-D 实际使用的三维
EdgeStereoSE3ProjectXYZOnlyPose 均将**整条边**的信息阵乘以 w 一次。
右相机/IMU/粗定位/重定位/BA 不启用加权。

本地 `Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp::RobustKernelHuber::robustify`
定义 rho(e)=e 或 2*delta*sqrt(e)-delta²。因此将delta改为sqrt(w)*delta，得到
`rho_scaled(w*e)=w*rho_original(e)`，不改变原残差空间中的鲁棒转折点。
本地实现把delta²缓存为float，实际等价关系受该单精度舍入影响；合成测试使用
1e-6相对/绝对混合容差，而非要求双精度逐位相等，不修改第三方g2o实现。
仍按原第3轮后去掉鲁棒核。

四轮优化后的每个重新分类位置均使用 `edge.chi2()/w` 与原5.991/7.815比较，
不允许低权重使极大原残差通过门限。原 PoseOptimization 并无额外正深度判断，
本次不添加/删除原外点条件；计算权重时预测非正深度只中性回退。
w=1退化为原优化，原内点成功门槛不放宽。

## 日志与原因

启用日志时产生唯一 `static_quality_<timestamp>.csv`，每RGB-D输入一行。
字段：frame_id、timestamp、quality_applied、skip_reason、allowed_matches、
depth_quality_valid、neutral_weight_count、weight_min、weight_mean、weight_sum、
effective_sample_size、final_inliers、quality_compute_ms。
allowed_matches只统计本次实际建立视觉边的非空关联；是初始图的观测集，包含随后判为外点的观测。
`Neff=sum(w)^2/sum(w²)`，不是最终内点数或新关键帧门槛。
final_inliers沿用Tracking最终长期地图内点口径，未修改其成功条件。
quality_applied表示有可靠粗快照可供评价；是否有有效深度仍看depth_quality_valid，不能混淆。
初始化/没有精优化的帧记录0观测、空统计与FINAL_OPTIMIZATION_NOT_RUN；精细支撑不足记录对应原因。
无观测的min/mean/Neff为空，sum=0，避免除零。

点级原因保存在Frame.mvStaticQualityReason：NOT_EVALUATED、DEPTH_EVALUATED、
COARSE_FALLBACK、NO_COARSE_SNAPSHOT、COARSE_POSE_UNRELIABLE、DEPTH_INVALID、
PREDICTION_NONFINITE、PREDICTION_BEHIND_CAMERA、MAPPOINT_BAD、WEIGHT_NONFINITE。
没有增加逐特征终端输出或改变B2/B4日志。

## 构建和运行

```bash
cd /home/aaa/ORB-SLAM3_original0
cmake -S . -B build_static_quality -DCMAKE_BUILD_TYPE=Release
cmake --build build_static_quality --parallel 2
```

内存紧张时先单独构建 Optimizer 对象，使用 GCC GC 参数降低编译内存峰值，
不修改O3或算法；完整构建的最大并行度仍为2。该环境有RealSense缺失警告，
与RGB-D文件输入无关；不自动安装系统软件。

本机低内存构建实际使用的补充命令（配置完成后，先等单对象构建结束，再全目标构建）：

```bash
task_quality_flags=$(sed -n 's/^CXX_FLAGS = //p' build_static_quality/CMakeFiles/ORB_SLAM3.dir/flags.make)
make -C build_static_quality -f CMakeFiles/ORB_SLAM3.dir/build.make \
  CMakeFiles/ORB_SLAM3.dir/src/Optimizer.cc.o -j2 \
  "CXX_FLAGS=${task_quality_flags} --param=ggc-min-expand=10 --param=ggc-min-heapsize=32768"
cmake --build build_static_quality --parallel 2 -- \
  "CXX_FLAGS=${task_quality_flags} --param=ggc-min-expand=10 --param=ggc-min-heapsize=32768"
```

```bash
python3 tools/validate_static_quality.py \
  --build-dir build_static_quality \
  --output build_static_quality/reproduce_01 --full --repeats 3
```

输出目录必须不存在。运行器先做合成及现有策略/有效期回归，之后短片段再完整序列；
新结果保存命令、配置、commit、源码/运行库/模型哈希、原始日志、轨迹、地图准入审计。
仅用户当前HOME下实际检查存在的数据/关联/evo才运行；缺失时保留明确跳过项。
两个序列各100帧off/on短测后，同组配置完整重复3次；关闭所有窗口/实时节奏。
ATE使用SE(3)、无尺度校正、0.02秒；RPE相邻帧all_pairs、米/度。
测试集真值只用于离线评价，绝不输入权重。参数调整集应与最终评估集分开，
这里的两个序列重复实验不构成独立泛化验证。

完成后在另一新目录生成统计和审计：

```bash
python3 tools/summarize_static_quality.py build_static_quality/validation_03 \
  --output build_static_quality/validation_03/report
```

标准差采用总体标准差(ddof=0)。frame_weight_distributions.csv给出逐帧权重最小值、
均值与Neff等摘要的分布，**不是逐特征权重直方图**。不使用系统自己的S/P/D标签作为运动真值。

单次手动运行可使用已验证的本机路径：

```bash
cd /home/aaa/ORB-SLAM3_original0
task_run=$(mktemp -d /home/aaa/ORB-SLAM3_original0/build_static_quality/manual_on_XXXXXX)
cp configs/static_quality/rgbd_tum3/quality_on.yaml "$task_run/effective.yaml"
cd "$task_run"
LD_LIBRARY_PATH=/home/aaa/ORB-SLAM3_original0/build_static_quality/lib:/home/aaa/ORB-SLAM3_original0/third_party_ort/onnxruntime-linux-x64-1.17.3/lib:/home/aaa/ORB-SLAM3_original0/Thirdparty/DBoW2/lib:/home/aaa/ORB-SLAM3_original0/Thirdparty/g2o/lib \
QT_QPA_PLATFORM=offscreen LIBGL_ALWAYS_SOFTWARE=1 \
/home/aaa/ORB-SLAM3_original0/build_static_quality/bin/ablation_sequence rgbd \
  /home/aaa/ORB-SLAM3_original0/Vocabulary/ORBvoc.txt "$task_run/effective.yaml" \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt 0 0 0 >run.log 2>&1
```

关闭组复制quality_off.yaml，必须使用另一个新目录。首版权重可能因粗位姿偏差、
地图点误差或非独立深度/虚拟右目观测而降低正确约束的贡献；有效样本数下降也可能
导致ATE升高。中性回退不会解决深度缺失。合成/编译通过不等同于实际精度提升。
不宣称消除漏检、不宣称满足论文发表条件。

## 验证记录

实际构建、合成测试、序列结果完成后在此追加；尚未完成的项目不记为通过。

### 构建与合成测试（2026-11-06）

隔离目录 build_static_quality 全目标构建和最终增量构建成功（并行度2）。
Optimizer 大文件先单独构建并使用 GCC GC 参数；其余源码保持Release/O3。
原版 Examples/RGB-D/rgbd_tum 和根lib/libORB_SLAM3.so的SHA256核对通过。

static_quality_test调用真实优化器并通过：参数拒绝、权重单调/有界、真实MapPoint/粗位姿/
主Frame深度权重准备、无效深度/非有限预测/相机后方/不可靠粗位姿的中性处理、
复制/赋值/reset/错误尺寸时Prepared失效、禁止特征不恢复、冻结权重不更新、
全1路径一致性、原入口忽略非1权重、二维与三维低权重外点仍拒绝、Huber缩放。
合成带偏差三维观测实验的平移误差：普通优化0.00117139m，加权8.4694e-06m；
这是受控合成输入结果，不是TUM定位精度提升。

保留失败记录：synthetic_first.log中Huber断言原容差过严；核查float缓存后只修正
测试容差，synthetic_final.log通过。validation_01中的双目拒绝测试错误使用RGB-D
相机配置，先在Settings缺少Camera2参数处退出；改用完整双目标定测试启动拒绝，
validation_02通过。没有修改第三方库、放宽外点门限或删除失败目录。

validation_02的legacy_checks/summary.json包含30项旧配置/策略/关联/有效期回归检查，
均通过。StaticQuality单独配置检查覆盖非法类型、NaN、参数范围、缺失模型的关闭组、
不完整Mode3拒绝、合法双目标定下明确拒绝启用、原版Mode0 RGB-D三帧启动。
实际数据集重复结果见完成后的 comparison.csv / aggregate.json；此处不提前宣称通过。

补充生命周期修正：Track()在新图/提前返回分支可能清空当前Frame，跳过优化的日志
改用进入Track前保存的输入frame_id与采集timestamp，而非读取空Frame的未设置字段。
因此validation_02的部分完整结果保留为阶段记录，不与后续固定版本三次重复合并。
这只修复日志元数据，不调整权重、运动阈值或位姿算法；增加实际System ResetActiveMap/
Reset三段输入的权重生命周期检查。完整Reset沿用原Frame编号重置规则，跨重置分析应
结合已有身份日志sequence_id区分会话，不能只用frame_id去重。

### 最终固定版本验证（validation_03）

39项顶层验证全部通过，其中legacy_suite另含30项旧配置/关联/策略回归检查。
实际System ResetActiveMap/Reset测试9帧通过：三段输入首帧均无旧有效权重。
四个100帧短测全部退出0、无失跟；之后两序列开/关各完整运行3次，共12次。
全部运行退出0，RECENTLY_LOST=LOST=0；sitting每次680条相机姿态/676个GT匹配，
walking每次827条相机姿态/826个GT匹配。两条轨迹均生成。
所有最终精细优化P/D/禁止项输入为0，最终地图审计禁止观测与索引错位均为0；
B4各frame/track无重复更新。关闭组不生成StaticQuality CSV。
程序与运行库哈希核对通过，原版可执行程序及根库SHA256核对通过。

ATE：SE(3)对齐、无尺度校正、t_max_diff=0.02s；
RPE：关联后相邻1帧、all_pairs、平移m/旋转deg。没有使用真值生成权重。

| 序列 | 开关 | 重复 | ATE RMSE(m) | ATE median(m) | 平移RPE RMSE(m) | 旋转RPE RMSE(°) |
|---|---|---:|---:|---:|---:|---:|
| sitting_static | off | 1 | 0.007009 | 0.005577 | 0.006386 | 0.175608 |
| sitting_static | on | 1 | 0.008929 | 0.007418 | 0.006758 | 0.182715 |
| walking_xyz | off | 1 | 0.282770 | 0.222482 | 0.017721 | 0.519734 |
| walking_xyz | on | 1 | 0.305653 | 0.253555 | 0.018085 | 0.510415 |
| sitting_static | on | 2 | 0.009107 | 0.007715 | 0.006481 | 0.177688 |
| sitting_static | off | 2 | 0.007732 | 0.006429 | 0.006240 | 0.178057 |
| walking_xyz | on | 2 | 0.020150 | 0.012145 | 0.014300 | 0.417988 |
| walking_xyz | off | 2 | 0.020266 | 0.012553 | 0.013782 | 0.416764 |
| sitting_static | off | 3 | 0.008825 | 0.006922 | 0.006248 | 0.171621 |
| sitting_static | on | 3 | 0.007704 | 0.006127 | 0.006862 | 0.182042 |
| walking_xyz | off | 3 | 0.020662 | 0.011939 | 0.014202 | 0.422005 |
| walking_xyz | on | 3 | 0.021363 | 0.012660 | 0.014932 | 0.426599 |

三次RMSE的中位数：

| 序列 | 开关 | ATE RMSE(m) | 平移RPE RMSE(m) | 旋转RPE RMSE(°) |
|---|---|---:|---:|---:|
| sitting_static | off | 0.007732 | 0.006248 | 0.175608 |
| sitting_static | on | 0.008929 | 0.006758 | 0.182042 |
| walking_xyz | off | 0.020662 | 0.014202 | 0.422005 |
| walking_xyz | on | 0.021363 | 0.014932 | 0.426599 |

walking的ATE总体标准差：off=0.123652m、on=0.134302m，两组都存在一次明显漂移。
因此不能只看中位数或最好一次，不能据此宣称新模块提高精度。
sitting的ATE标准差：off=0.000746m、on=0.000624m；当前开启组的中位数更差。
本轮结果不支持默认启用；源码默认保持0，实验on配置仅用于继续验证。

权重计算每帧平均耗时的三次中位数：sitting=0.027472ms、walking=0.031850ms。
这只是PrepareStaticQuality耗时，不含全部优化/日志和多线程调度影响，
不能由总Tracking耗时差宣称模块加速。
有效深度质量观测占比的三次中位数：sitting=89.25%、walking=65.68%；
包括中性权重的全观测平均权重三次中位数：0.694208/0.677376。

结果目录：build_static_quality/validation_03/

- comparison.csv：全部短/完整运行、每次ATE/RPE、匹配数量、耗时和质量统计。
- report/aggregate.csv：三次中位数、均值、总体标准差及范围。
- report/frame_weight_distributions.csv：逐帧摘要分布，不是逐特征直方图。
- report/invariant_audit.csv、report/audit.json：12次最终审计。
- 各run目录：有效配置、命令、commit、模型/配置哈希、run.log、B2/B3/B4 CSV、
  粗/精策略与地图准入日志、两条轨迹、ATE/RPE txt与zip、StaticQuality CSV(仅on)。
- implemented_sources.tar.gz、source_diff.patch、source_hashes.json、runtime_hashes.json：
  固定构建对应的未提交源码、补丁和哈希；不是只依赖HEAD推测当前实现。

### 诊断与剩余限制

这套公式未显式建模粗位姿或MapPoint不确定度，可能把正确深度与错误预测之间的差异
误当作低质量；对整条RGB-D三维边降权也会同时削弱正确的二维定位约束。
walking约34%～36%的关联使用中性权重1，不能把这些缺深度观测视为高质量。
本轮未定位漂移的单一因果来源，不应把全部变化归因于权重或线程随机性。

已有日志的关联现象：walking大漂移两组的精细内点均值约185/188、活动关键帧281/280；
其余walking运行精细内点均值约220～225、活动关键帧154～163。
这是诊断线索，不是证明；需要进一步按背景/S区域和深度分层检查残差与权重，
并在独立验证集标定A/B、冻结后再做留出评测。本轮未进行参数搜索或利用测试真值调权。
仅轴向深度一致性不能覆盖所有横向运动，也不能修复身份、S状态或原始非有限地图几何的错误。

未验证：独立调参/留出集泛化、真实困难场景自然失跟/重定位/新图恢复、ASan/TSan，
以及单目/双目/惯性真实序列（新模块本就禁止在这些传感器上启用，仅编译兼容并测试旧入口）。
所有测试保留原B2/B3/B4规则与后端目标函数；默认关闭可直接保留旧算法路径。
