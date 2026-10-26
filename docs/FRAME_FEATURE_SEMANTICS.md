# 第 4 批：主 Frame 特征语义属性（仅标注）

## 数据与索引

`include/FeatureSemanticTypes.h` 定义 SemanticRegion、TemporalState、FeatureUsePolicy。
`Frame` 增加用户要求的七组数组，长度始终为 N。所有实际构造器在 ORB
完成并设置 N 后初始化；空特征也初始化为零长度。默认 Frame 的 N 现在为 0。
复制构造和原有赋值都保留元数据，既不排序也不删改原有特征。

本批实际标注 RGB-D / IMU RGB-D；单目与双目只初始化 UNKNOWN 元数据，
不修改其算法。普通 RGB-D 的 i 对应 mvKeys[i]、mvKeysUn[i]、描述子第 i 行、
mvpMapPoints[i] 和新增数组。鱼眼双目的原有特征组织保持不变：N 是左右
描述子拼接总数，右目索引为 Nleft+j，而不是把 mvKeys 强行扩成 N。

## 开关与参数

所有字段由已有 DynamicDetector YAML 读取器解析，没有第二套运行时解析器：

```
FeatureSemantic.Enable: 1
FeatureSemantic.SafeBoundaryDistancePixels: 3.0
FeatureSemantic.DepthEdge.Enable: 1
FeatureSemantic.DepthEdge.Radius: 1
FeatureSemantic.DepthEdge.AbsoluteMeters: 0.05
FeatureSemantic.DepthEdge.Relative: 0.02
FeatureSemantic.ShowOverlay: 1
```

Enable 的默认值为 0，其他默认值如上。旧配置不开启标注；开启需要显式
DynamicFrontend.Mode=2。本批不允许将此配置用于双目入口。新建完整配置为：
`configs/frontend_modes/rgbd_tum3/mode2_feature_annotations.yaml`，原有 YAML 未覆盖。
相机、ORB、B2/B3 阈值和 B4 的 3/6/10/15 均沿用旧 Mode 2。

深度边缘阈值是初始的标注参数，不是运动判定阈值，也没有替换 B2 的任何参数。

## 标注规则与调用位置

```
YOLO -> 统一实例关联 -> B2 独立候选 -> 主 Frame 整图 ORB
     -> AnnotateRGBDFeatures（读取上一时刻 B4）
     -> 完整 Track -> B2 -> B3 -> B4（只更新一次）
     -> 更新主 Frame 当前 B4 元数据 -> 日志 / FrameDrawer
```

使用 YOLO 原始逐实例 mask，不使用 B1 膨胀 mask。以 mvKeys 的原始像素坐标
查找，不使用去畸变后的 mvKeysUn。mask 尺寸必须与实际提取图像一致，否则报错。
深度使用 GrabImageRGBD 已换算为 CV_32F 米制的图像。

- 不在任何 person mask 内：BACKGROUND。背景自身的深度边缘不冒称人物边界。
- 唯一归属人物、距掩膜边界严格大于 d_safe 且没有明显深度跳变：PERSON_INTERIOR。
- 人物 mask 内侧安全带，或人物内有效邻域深度跳变：PERSON_BOUNDARY。
- mask 重叠、track_id 无效、坐标无效或越界：UNKNOWN，实例/track ID 为 -1。

边界距离通过补零边框后的精确 L2 distanceTransform 计算，图像边缘截断的人体
也具有边界。深度跳变条件是邻域有效正深度与中心深度之差超过
max(AbsoluteMeters, Relative * centerDepth)。0/NaN/Inf 忽略；深度缺失本身
不改变已知语义归属。外侧像素仍按用户“mask 外为背景”规则标为背景，本批不
额外创建膨胀安全带。所有这些参数仅影响标签/颜色。

已有 B4 轨迹通过 const GetState 读取；新 ID 的初始历史为 P，无法归属为 UNKNOWN。
B4 本帧完成后只刷新 mvCurrentB4State，不覆盖 mvPreviousB4State，不再调用状态更新。
背景没有 person 的 B4 状态，保持 UNKNOWN。

所有特征的 mvFeaturePolicy 保持 USE_NORMAL，mvDynamicExcluded 保持 false；
包括 P/D、边界、UNKNOWN 都不剔除。标签、历史状态、当前状态不得被此批代码
用于匹配、位姿优化、关键帧或地图点准入。

## Viewer

FrameDrawer 使用线程安全复制后的元数据给主 Frame ORB 实心点着色：

- 青色：背景；蓝色：人体内部；橙色：人物边界；灰色：UNKNOWN。
- 底部有 MAIN ORB 图例。B2 独立候选仍沿用原有空心圈，B4 框仍显示 S/P/D。
- FeatureSemantic.ShowOverlay=0 可隐藏新颜色，但保留标注。

渲染函数 DrawSemanticFeatures 同时用于实时 Viewer 与无 GUI PNG 测试。

## 验证与复现

构建使用隔离 build_yolo11，所有命令最多 `-j2`。先单独构建重型对象，避免
上一批观察到的内存竞争：

```
cmake -S . -B build_yolo11
cmake --build build_yolo11 --target src/Optimizer.o -- -j2
cmake --build build_yolo11 --target ablation_sequence feature_semantic_test frontend_mask_test b4_state_machine_test instance_track_lifecycle_test b2_candidate_test yolo_seg_smoke rgbd_tum stereo_euroc mono_tum -- -j2
python3 tools/run_frontend_stage_validation.py --stage batch4_feature_annotations --mode-config 2=configs/frontend_modes/rgbd_tum3/mode2_feature_annotations.yaml
python3 tools/audit_feature_semantics.py <本次结果目录> --reference results/frontend_stage_validation/step5_shared_track_20261022_153908
python3 tools/audit_shared_track_updates.py <本次结果目录>
```

feature_semantic_test 检查边界、深度台阶、重叠 UNKNOWN、无效坐标、空帧、复制、
单目默认元数据、掩膜尺寸错误，以及标注前后原特征/描述子/地图点/外点/位姿不变。
frontend_mask_test 额外检查普通双目、鱼眼左右空特征时数组长度。

在线每帧打印 `[FEATURE SEMANTICS]`，核查各类数量之和等于 N、normal=N、
excluded=0、aligned=1。与标注关闭的上一批逐帧比较 FAST/ORB 数量；共享 ID
检查同时验证 B4 不会被读取历史状态的操作多更新一次。

这些是元数据和功能正确性检查，不代表新的精度或性能结论。多线程调度仍可
改变轨迹和关键帧数量，不要求开启标注前后轨迹逐位相等。

## 本次验收结果（2026-10-22）

新增/修改的主要位置：

- Frame.h / Frame.cc：枚举类型引用、七组数组、初始化和复制、长度核查。
- FeatureSemanticTypes.h：类型与默认配置；FrameSemanticAnnotator.h/.cc：标注和当前状态回填。
- DynamicFrontendConfig.h / DynamicDetector.cc：同一 YAML 模块读取开关与参数。
- B4TemporalStateMachine.h/.cc：只读 GetState，不修改转移规则、计数或 TTL。
- Tracking.cc：Frame 构造后标注、B4 后元数据回填与运行时不剔除断言。
- FrameDrawer.h/.cc：主 ORB 语义颜色和图例；CMakeLists.txt：隔离测试目标。
- feature_semantic_test.cc / frontend_mask_test.cc：单元与多相机兼容测试。
- audit_feature_semantics.py：逐帧标签/提取数量核查。
- run_frontend_stage_validation.py：中断后可引用已通过且配置/二进制哈希一致的短测，完整重试使用新目录。

构建最终成功，所有构建命令使用 -j2。初次多目标构建在 CMake 自动更新后未能
在旧的父 make 进程中识别新测试目标；重试后修正了测试代码的 C++ 初始化解析
歧义。完整失败/成功构建日志保留于
`results/frontend_stage_validation/batch4_feature_build_20261022_01/`。
五项单元/回归测试及 16 项原模式启动检查均通过。

真实 RGB-D 主 Frame + YOLO 预览（已检查颜色与人体位置对应）：
`results/frontend_stage_validation/batch4_feature_build_20261022_01/preview/walking_main_orb_semantics.png`。
PNG 使用与 FrameDrawer 相同的绘制函数；本次没有打开交互式 Pangolin/Current Frame 窗口。

最初在线验证目录为 `results/frontend_stage_validation/batch4_feature_annotations_20261022_161704/`。
两个 100 帧短测成功，但验证驱动随后 SIGTERM（143），完整 sitting 停在 frame 90，
原因未确认、无正常退出码或轨迹输出；未将它算作成功完整运行。该目录和中断
说明原样保留。随后在新目录受控重试，引用原来已通过的两项短测：
`results/frontend_stage_validation/batch4_feature_annotations_retry01_20261022_161954/`。

| 完整序列 | 输入/轨迹行 | KeyFrame 行 | 退出码 | RECENTLY_LOST / LOST |
|---|---|---|---|---|
| sitting_static | 680 / 680 | 14 | 0 | 0 / 0 |
| walking_xyz | 827 / 827 | 314 | 0 | 0 / 0 |

语义数量是跨帧特征观测之和，不是不同物理点个数：

| 完整序列 | 背景 | 人体内部 | 人体边界 | UNKNOWN | 剔除 |
|---|---|---|---|---|---|
| sitting_static | 469415 | 164192 | 51057 | 0 | 0 |
| walking_xyz | 700811 | 102324 | 29299 | 62 | 0 |

`feature_semantic_audit.json`：所有帧数组/主特征对齐、各类计数和为 N、策略全为
USE_NORMAL、剔除为零；相对第 5 步关闭标注的参考，逐帧 FAST/ORB 数完全一致。
`shared_track_audit.json`：共享 ID 一致，最大更新次数为每帧每 track 一次；完整
序列中的 984 次不可靠证据（247+737）均冻结状态及 Ns/Nd/Nr。
没有自动提交 Git，没有改变 B2/B3 阈值、B4 转移、匹配、优化或地图点准入。
