# 多类别动态 RGB-D 前端

本次在 968bacd 上扩展已有语义—几何—时间状态策略，不接入 SuperPoint、异步推理、深度质量检查或新优化器。没有提交或推送代码。

## 修改位置

- `include/Deep/SemanticClassConfig.h`：共享类别选择、管理权限与显示名称。
- `include/Deep/DynamicDetector.h`、`src/Deep/DynamicDetector.cc`：在原YAML读取链路解析、校验、打印并传递同一份类别配置；记录生效类别；修正原先写死为0的MapAction启动日志。
- `include/B3SemanticGeometryFusion.h`、`src/B3SemanticGeometryFusion.cc`：选定类别的语义先验，不改变几何公式与三区域阈值。
- `include/Deep/B4Config.h`、`src/B4TemporalStateMachine.cc`：多类别证据准入检查，复用原S/P/D、有效期和TTL。
- `include/FeatureSemanticTypes.h`、`src/FrameSemanticAnnotator.cc`：主Frame按选定类别标注，原索引级粗/精/地图策略因此对新增类别实际生效。
- `src/Tracking.cc`：统一配置传给B3，检查B2与语义类别/身份一致，Viewer显示实际类别。
- `include/YOLOSeg.h`、`src/YOLOSeg.cpp`：澄清旧字段名、打印模型实际类别元数据；沿用已有逐类别NMS和逐实例掩膜解码。
- `src/Deep/MaskPreview.cc`：去除窗口里写死的人体名称；标记为分割而非错误暗示EvidenceOnly也在剔除。
- `CMakeLists.txt`、`tools/multiclass_frontend_test.cc`、`tools/validate_multiclass.py`、`tools/yolo_seg_smoke.cc`：新增测试、配置/真实短测审计及逐实例类别诊断。
- `.gitignore`：排除新独立构建目录，避免把库、二进制和测试结果上传为源码。
- `tools/run_multiclass_sequence.sh`、`configs/multiclass/rgbd_tum3/*.yaml`：隔离运行工具及三组完整配置。

`Optimizer.cc`、`LocalMapping.cc`、`MapPoint.cc`和BA目标函数未改动；地图准入复用已有实现，不在通用MapPoint函数中增加类别拦截。

## 配置与兼容性

```yaml
Semantic.MultiClass.Enable: 1
Yolo.NumClasses: 80
Yolo.InputWidth: 640
Yolo.InputHeight: 640
Yolo.DynamicClasses: [0, 1, 2, 3, 5, 7, 15, 16]
Semantic.SelectedClassNames: ["person", "bicycle", "car", "motorcycle", "bus", "truck", "cat", "dog"]
```

`Semantic.MultiClass.Enable` 默认 0。旧 YAML 不需要改变：原 person 判定、B2 数学公式、B4 3/6/10、有效期、BA 和原相机参数保持原有设置。旧配置选择非 person 类时会警告：仅有检测掩膜不代表已打开多类别策略。

`Yolo.DynamicClasses` 是唯一选择列表。多类别开启时，列表内类别都是“可能运动的语义先验”，不是运动真值。B3 将选定类别和可靠几何证据结合，B4 按实例独立更新 S/P/D。未选择类别不受本机制管理；这不是开放类别运动检测。

`Semantic.SelectedClassNames` 可省略，此时日志用 person / class_N；如设置，须与 ID 列表逐项对应。名称只用于显示，不决定算法。模型加载时打印实际 `names` 元数据（若存在），必须核对模型的类别顺序。现有原始分割解码器校验输入形状、FP32 类型、输出通道和类别数；不能只改类别数来支持任意 ONNX 导出格式。

`Yolo.NumClasses` 默认80，输入尺寸默认640×640。不下载、训练或替换模型。新增模型不具备的类别，需要另行训练/导出兼容模型，修改 YAML 不能创造识别能力。

## 实际顺序和动作

RGB-D → YOLO选定类别逐实例掩膜 → 同类别身份关联 → B2独立候选ORB + 主Frame全图ORB → 用原始实例掩膜标注主特征及历史状态 → 粗位姿 → B2 → B3 → B4 → 当前精细策略 → TrackLocalMap/位姿优化 → 关键帧决策 → 索引级地图准入 → LocalMapping。

模式3中，背景和S可使用；新实例P、D、边界、无法归属实例的特征受已有粗/精策略约束。精细过滤在进入局部跟踪前及局部搜索后执行；地图准入沿用 Tracking、KeyFrame、LocalMapping 和融合的索引级检查，禁止相应特征新建地图点/长期观测，不删除历史地图点。粗阶段的原有失败回退仍保留，不能把策略存在描述为永不使用回退匹配。

保持 `SemanticRegion::PERSON_INTERIOR/PERSON_BOUNDARY` 的数值与序列化不变；开启多类别时这两个历史名称表示“受管理目标内部/边界”。`B2PersonInstance`、`persons`、`personMask` 等是兼容字段名，现在可容纳所有选定类别。没有删除关键点或描述子行，Frame/KeyFrame索引关系不变。

`B2.EvidenceOnly=1`、`B3.EvidenceOnly=1` 是其只读接口，不意味着整个前端不生效。动作组须为 `DynamicFrontend.Mode=3`、`FeaturePolicy.EvidenceOnly=0`、`B4.EvidenceOnly=0`，并开启粗/精过滤及地图准入。纯诊断组用 Mode=2 和全部 EvidenceOnly=1，不施加特征/地图动作。

## 完整实验配置

- `configs/multiclass/rgbd_tum3/person_only_action.yaml`：person旧行为对照。
- `configs/multiclass/rgbd_tum3/selected_classes_action.yaml`：person、车辆及部分动物的动作组。
- `configs/multiclass/rgbd_tum3/selected_classes_evidence.yaml`：同样类别的纯证据组。

三组基于同一 TUM3 标定。其他 RGB-D 数据集必须使用自己的内参、图像尺寸、颜色顺序、深度尺度、深度有效范围和时间关联，不能直接套用 TUM3 标定。

## 构建和运行

构建目录 `build_multiclass`，最终构建 `cmake --build build_multiclass --parallel 2`。Optimizer.cc 若峰值内存过高，先单独编译该目标，再双线程构建其他目标；不修改优化器源码或优化等级。输出只在该目录，不覆盖原 Examples 可执行文件或原 lib。

```bash
cd /home/aaa/ORB-SLAM3_original0
# 先100帧，再把100改为0完整运行；每次自动创建独立结果目录，关闭所有窗口。
bash tools/run_multiclass_sequence.sh multi \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt 100
# 第一个参数改为person或evidence，运行对照/诊断组。
```

运行目录保存生效 YAML、commit、Git状态、已有跟踪文件差异、命令、二进制/库/YAML哈希、日志、退出码、轨迹，以及配置打开的B2/B3/B4和特征/地图策略CSV。新增未跟踪源码仍应连同本交付保留，Git diff 本身不包含未跟踪文件。

## 验证与局限

`build_multiclass/bin/multiclass_frontend_test` 使用合成 car/dog 掩膜和几何证据，检查 B2保留类别、B3先验、分实例B4转移、类别隔离、旧行为、粗策略、精细优化排除、KeyFrame地图准入和索引不变。

TUM短测只能检查真实RGB-D整链路稳定性，不能证明车辆或动物运动识别正确。非person真实动态数据、类别检测精度、身份真值、静态保留/动态泄漏、完整序列ATE及多次重复性能需要另行评测。合成测试或编译成功不代表精度提升。

原B2深度上限5m、候选数量及可靠性阈值主要来自现有室内实验；远距离车辆、低纹理物体、细小动物可能长期证据不足。新增大量类别会减少背景参考和静态支撑，也增加逐实例处理成本。应先固定验证集评估，再按类别范围扩展；不要为看到更多MOVING而任意降低阈值。

## 本次实际验证记录

分支 `snapshot_clean`，基础提交 `968bacd072c4af2912f6660020a8410e8e3964b0`，新增改动未提交。

- 双线程最终构建成功：`build_multiclass/build_verify.log`。首轮构建在新增测试中发现向Frame非const引用传入临时Mat的问题，改为具名变量后通过；首轮错误保留在 `build_multiclass/build.log`。既有Eigen弃用等警告未在本次扩展中处理。
- 实际模型输出 `output0=[1,116,8400]`、`output1=[1,32,160,160]`，输入 `[1,3,640,640]`，ORT 1.17.3；80类元数据与所选名称一致。记录见 `build_multiclass/model_inspection.log`。
- 新增检查13/13通过，报告 `build_multiclass/validation_multiclass_01/summary.json`。包含合成集成测试、3组配置、7组非法配置和2个真实短序列；非法配置预期退出2，不是未处理失败。
- 旧回归30/30通过，报告 `build_multiclass/validation_regression_01/summary.json`：15项单元/集成测试、A0至A3配置、默认/非法配置、ORIGINAL/B1/旧EvidenceOnly的3帧启动。ORIGINAL配置使用不存在模型路径，确认关闭时不加载模型。
- sitting_static前100帧：OK=100，LOST=0，100条Camera轨迹、5条KeyFrame轨迹。精细优化禁止输入=0，地图禁止观测=0，索引错位=0。
- walking_xyz前100帧：OK=100，LOST=0，100条Camera轨迹、28条KeyFrame轨迹。同样两项禁止计数及索引错位均为0。
- 原 `Examples/RGB-D/rgbd_tum` 和 `lib/libORB_SLAM3.so` 与修改前SHA256一致；新程序链接独立构建目录中的库。

真实短测未提供非person运动真值。sitting第41、42、58帧共出现4条car实例输出，置信度约0.255至0.280，框很小，均为FUSION_UNRELIABLE/P；需人工核查是否误检。不能把这些输出解释为已验证车辆检测或运动识别成功。新增类别的误检即使保持P，也可能排除原本背景特征，需纳入静态保留率和ATE消融。

复现检查：

```bash
python3 tools/validate_multiclass.py --build-dir build_multiclass \
  --output build_multiclass/validation_multiclass_02 --short-sequences
python3 tools/validate_instance_association.py --build-dir build_multiclass \
  --output build_multiclass/validation_regression_02 --compatibility
```

输出目录必须不存在。未执行非person动态真实序列、完整序列ATE/RPE和重复精度实验；不宣称本次扩展提升定位精度。全链路验证限定RGB-D，不能直接推广为已适配双目/单目B2几何前端。
