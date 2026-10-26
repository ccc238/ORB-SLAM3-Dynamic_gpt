# 当前工程 YOLO11s-seg 接入与消融实验

实施日期：2026-10-19。源代码起点为 commit
`fe5b4ade86b247580cc9565e0b21d86a2b81faaf`。用户原有 `build.sh` 暂存修改、
`docs/yolo11_before/` 和 `third_party_ort/` 均保留。

## 实现范围

仅 Stereo、RGB-D（含其已有惯性分支的接口兼容）接入同步分割。
`GrabImageMonocular`、单目 Frame 构造函数及单目初始化算法未增加分割逻辑。
模型由 Tracking 持有的 DynamicDetector 加载一次；双目用同一会话顺序推理左右图。

处理顺序为：

```
System 的原有校正/缩放
→ Tracking::GrabImageStereo / GrabImageRGBD
→ DynamicDetector：仅启用时颜色适配 → YOLOSeg 原始掩膜
→ MaskPostProcessor：可选一次椭圆膨胀
→ Frame 左/右掩膜
→ ORBextractor：FAST 候选 → 非零掩膜过滤 → OctTree → 方向 → 描述子
→ 原有匹配、跟踪、建图、回环
```

掩膜与实际交给提取器的输入图像同尺寸，类型为 `CV_8UC1`，255 剔除、0 保留。
候选点尚在去边框坐标中，查询掩膜时补回 border，再乘该层 scale。
过滤按原输入坐标执行，不拿去畸变后的 keypoint 查询掩膜。

## 运行时开关

| 配置 | 默认值 | 行为 |
|---|---|---|
| `Yolo.Enable` | 0 | 关闭时不读模型路径、不创建 Ort::Env/Session、不做 YOLO 颜色转换、不生成掩膜，返回空 Mat |
| `MaskPostProcess.Enable` | 0 | 开启需 `Yolo.Enable=1`，否则立即抛配置错误 |
| `MaskPostProcess.DilateRadius` | 3 | 椭圆核直径 `2r+1`，每张掩膜一次迭代；r=0 不改变掩膜 |
| `Yolo.SaveMask` | 0 | 独立保存实际交给 Frame 的掩膜 |
| `Yolo.SaveOverlay` | 0 | 独立保存叠加图，绘制在副本上 |
| `Yolo.LogTiming` | 0 | 逐视图推理/后处理耗时、像素数日志 |
| `Yolo.ShowMask` | 0 | 实时二值掩膜与叠加窗口；需启用 YOLO 和可访问的图形桌面 |
| `Experiment.FrameLog` | 0 | 新实验驱动的逐帧总跟踪耗时/状态 CSV |

所有 Enable/保存/显示开关严格接受整数 0/1。正式测速时关闭保存、逐帧日志和实时显示。
实时显示也应关闭，不与磁盘保存开关绑定。
没有检测到 person 时，启用的 YOLO 返回全零掩膜；关闭 YOLO 则为空 Mat。
模型加载、输出形状和推理错误均抛出明确异常，不自动退回 baseline。

固定 person 默认类别 `[0]`，置信度 0.25、NMS IoU 0.45、mask threshold 0.5、ORT intra-op 线程数 2。
`Yolo.DynamicClasses` 可配置，但本次生成的组均保持 `[0]`。

旧 `Yolo.DilateRadius` 仅在所有新 `MaskPostProcess.*` 键均缺省时迁移：
半径继承旧值；当 YOLO 开启且旧半径大于 0 时启用后处理。
YOLO 关闭时旧配置保持后处理关闭。迁移会在日志中明确提示。
新旧键同时出现无论数值是否相同均报冲突，要求删除旧键，避免隐含双重膨胀。

## 本地模型与依赖

- 模型：`/home/aaa/models/yolo/yolo11s-seg.onnx`，未下载、替换或重新导出。
- SHA256：`205e029a8be9f835cf798bb56cd95e8927b56b359b37dbe400bfee777adb808d`。
- ORT C++：`/home/aaa/ORB-SLAM3_original0/third_party_ort/onnxruntime-linux-x64-1.17.3`。
- 实测输入：`images`，FP32 `[1,3,640,640]`。
- 实测输出：`output0` FP32 `[1,116,8400]`、`output1` FP32 `[1,32,160,160]`。
- 解析原始分割头，无额外 objectness，80 类、32 个原型系数；CPU EP。
- 复用原来工作的 OpenCV、Eigen、Pangolin、DBoW2 和 g2o，不安装新的系统依赖。

## 与参考文档的调整

1. 关闭路径返回空 Mat，颜色转换移入已启用分支；参考代码的禁用全黑掩膜没有沿用。
2. 从 YOLOSeg 完全移除膨胀参数和实现，独立为 MaskPostProcessor。
3. 增加严格的开关、依赖与配置迁移检查。
4. 构造模型时就校验真实张量布局并打印接口；错误不隐式吞掉。
5. 不整体覆盖 Tracking/Frame 原文件，只改所需调用和参数。
6. CMake 复用已存在的 g2o/DBoW2，不执行会写入原库目录的 add_subdirectory(g2o)。
7. 新构建所有目标（含原单目和 old 示例）的输出都锁定到新 build 目录。
8. 保存掩膜时每个视图独立一行 index.csv，同一左右对共享 frame/timestamp；
   有别于参考文档的一对一行宽表，但保留明确的 left/right 配对。
9. 新增独立 headless 实验入口和拒绝覆盖目录的 Python 包装器。

## 修改版统一稳定性修复

三个修改版组均包含：

- 针孔双目：无描述子、无匹配距离集合时提前返回，深度保持无效。
- 鱼眼双目：重叠区描述子不足时提前返回，一侧空描述子时安全拼接。
- 双目/RGB-D 在空特征提前返回前初始化 IMU mutex 和左右特征标记，避免复制未初始化成员。
- ORB：清空复用输出关键点、空候选提前返回、小图网格和竖图 OctTree 至少一个分区。
- Shutdown：请求后台线程结束并等待，join LocalMapping/LoopClosing；有 Viewer 时
  请求并等待其结束，避免自己 join 自己；重复 Shutdown 直接返回。
- TUM 轨迹导出：当前地图没有关键帧时明确报错，防止索引空集合。

这些修复没有改变正常分支的匹配阈值、关键帧策略、BA、回环或优化设置。
但它们以及编译标准/新增调用开销意味着“修改版全部关闭”不等于原版二进制基线。
原版库、可执行文件必须继续单列为独立实验组，不使用新库运行原版入口。

## 构建与配置

```bash
cd /home/aaa/ORB-SLAM3_original0
cmake -S . -B build_yolo11 -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/home/aaa/ORB-SLAM3_original0/third_party_ort/onnxruntime-linux-x64-1.17.3
cmake --build build_yolo11 --target ablation_sequence yolo_seg_smoke frontend_mask_test \
  rgbd_tum stereo_euroc stereo_kitti stereo_tum_vi mono_tum -j1
```

新库 `build_yolo11/lib/libORB_SLAM3.so`；新程序 `build_yolo11/bin/`。
禁止在旧 `build/` 重新配置；CMake 对此有显式检查。
旧 `lib/`、Examples 下的二进制和两个第三方库可用既有 SHA256 清单核对。

六份完整配置：

```
/home/aaa/ORB-SLAM3_original0/configs/ablation/rgbd_tum3/baseline.yaml
/home/aaa/ORB-SLAM3_original0/configs/ablation/rgbd_tum3/yolo_only.yaml
/home/aaa/ORB-SLAM3_original0/configs/ablation/rgbd_tum3/yolo_dilate.yaml
/home/aaa/ORB-SLAM3_original0/configs/ablation/stereo_euroc/baseline.yaml
/home/aaa/ORB-SLAM3_original0/configs/ablation/stereo_euroc/yolo_only.yaml
/home/aaa/ORB-SLAM3_original0/configs/ablation/stereo_euroc/yolo_dilate.yaml
```

三组内除了两个模块开关，其余配置完全相同。标定没有互用。
RGB-D 基于 TUM3.yaml，将三组的 Camera.RGB 一致设为 0，以匹配 OpenCV imread 的 BGR；
仅更正颜色顺序声明，未改变标定、深度比例或 ORB 参数。原 TUM3.yaml 保持原样。
EuRoC 配置保持原 Camera.RGB=1；输入为灰度，不参与颜色次序解释。
比较旧基线时应明确记录这项公共颜色修正，不能声称逐像素输入处理与旧 TUM3 基线相同。

## 运行与结果保存

RGB-D（首次可加 `--max-frames 60` 做流程检查；省略该参数跑完整序列）：

```bash
cd /home/aaa/ORB-SLAM3_original0
python3 tools/run_ablation.py --mode rgbd \
  --config configs/ablation/rgbd_tum3/yolo_only.yaml \
  --sequence /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  --associations /home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt \
  --output /home/aaa/ORB-SLAM3_original0/results/ablation/rgbd_yolo_only_01
```

EuRoC 双目（自动读取 cam0/cam1 data.csv，在结果目录生成匹配列表）：

```bash
python3 tools/run_ablation.py --mode stereo \
  --config configs/ablation/stereo_euroc/yolo_dilate.yaml \
  --sequence /home/aaa/Datasets/EuRoc/MH_01_easy \
  --output /home/aaa/ORB-SLAM3_original0/results/ablation/stereo_yolo_dilate_01
```

替换组名和新的输出目录即可切换，不需要重新编译。默认无 Viewer；需要显示时加
`--viewer`，无物理显示器可从 `LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 ... --viewer` 启动。
默认保留原例程按时间戳 pacing 的行为；`--no-pace` 必须在同一比较中一致使用。

每次 `--output` 必须不存在，已有目录即拒绝运行。保存：

- `effective_config.yaml`：本次配置原文；
- `run_manifest.json`：实际命令、配置/输入列表/程序/模型哈希、开始时间、退出状态、墙钟耗时；
- `run.log`、`exit_code.txt`、`trajectory_line_count.txt`；
- `CameraTrajectory.txt`、`KeyFrameTrajectory.txt`；
- `tracking_timing.yaml`：Track 调用平均/中位耗时和状态计数；
- `semantic_timing.yaml`：生效模块状态、推理/后处理调用数与累计耗时；
- 可选 `frame_timing.csv`，可选唯一 `masks_<id>_<pid>/` 子目录。

`Yolo.OutputDir` 指向已经存在的父目录；默认 `.` 即本次结果目录。
计时中 Track 包括推理、后处理、特征、跟踪，启用图像保存时也包括写盘；
语义推理统计覆盖预处理+网络+原始掩膜解码，不包含 Tracking 灰度转换。
两次左右推理耗时均计入双目 Track；不要将单视图时间当成整对时间。

## 验证边界与后续扩展

实际检查记录见 `build_yolo11/validation/`；数据集短序列检查是流程验证，
不能替代完整序列 ATE/RPE、多次重复实验或精度提升结论。
鱼眼无特征边界使用合成数据检查，没有真实鱼眼序列精度验证；惯性模式仅接口兼容。
原单目示例编译检查也不等于单目运行验证。

后续新增模块：增加独立 `Module.Enable`（默认 0），在构造时验证依赖，
在处理函数开头为关闭分支直接返回；把生效状态和耗时加入结果摘要，
在独立配置组仅改变该开关；如果模块改变图像尺寸/坐标，必须在掩膜送入 Frame 前
完成同步变换。不要把新模块隐含塞进 YOLOSeg 或复用膨胀开关。

## 本次实际验证记录

自动检查命令（每次新建独立报告目录）：

```bash
python3 tools/check_yolo_integration.py
python3 tools/validate_short_sequences.py
```

实际自动检查包含：模型关闭且缺失路径、开启且缺失路径、依赖错误、旧配置迁移、
新旧配置冲突、原始掩膜/一次膨胀的逐像素比较、左右图独立推理、保存开关独立、
FAST 多层坐标过滤、空掩膜/全掩膜、双目空描述子/无匹配、鱼眼单侧空特征安全性、
配置组参数一致、单目入口与构造函数保持原文、原版二进制与库哈希不变。

最终检查报告：
`build_yolo11/validation/integration_20261019_111746_591876/results.json`。
独立构建完成：主库、ablation_sequence、frontend_mask_test、yolo_seg_smoke、
rgbd_tum、stereo_kitti、stereo_euroc、stereo_tum_vi，以及 mono_tum、mono_kitti、
mono_euroc、mono_tum_vi、mono_inertial_euroc、mono_inertial_tum_vi、
stereo_inertial_euroc、stereo_inertial_tum_vi。
RealSense SDK 本机未安装，其可选目标未构建；未为此安装或更换依赖。

数据集流程报告：
`/home/aaa/ORB-SLAM3_original0/results/yolo_validation/20261019_111425_079083/validation_summary.json`。

| 模式/组 | 退出码 | 相机轨迹行数 | 关键帧轨迹行数 | 推理调用数 | 后处理调用数 |
|---|---:|---:|---:|---:|---:|
| TUM3 baseline | 0 | 60 | 22 | 0 | 0 |
| TUM3 yolo_only | 0 | 60 | 13 | 60 | 0 |
| TUM3 yolo_dilate | 0 | 60 | 9 | 60 | 60 |
| EuRoC baseline | 0 | 60 | 11 | 0 | 0 |
| EuRoC yolo_only | 0 | 60 | 10 | 120 | 0 |
| EuRoC yolo_dilate | 0 | 60 | 10 | 120 | 120 |

六组均记录 OK=60，RECENTLY_LOST=0，LOST=0；这只是短序列运行检查。
额外的完整 SLAM 启动检查将 baseline 模型路径改为确定不存在的路径，10 帧正常退出，
导出 10/4 行轨迹；再次使用相同结果目录被拒绝，没有覆盖结果。

Viewer 的第一次附加检查在执行沙箱中失败（Pangolin 无法连接 X display，退出 -6），
日志保留于 `viewer_enabled_smoke/`。在可访问本机 Xvfb 的执行环境中、同样使用软件 OpenGL
重试成功：`viewer_enabled_retry/`，退出 0，导出 10/4 行轨迹。
重试没有因此改变 YOLO/SLAM 算法代码。

本次没有完整序列 ATE/RPE、统计显著性、多次重复或真实性能基准结果；
没有真实鱼眼或 IMU 数据集运行验证，也没有单目数据集运行验证。

原版独立基线仍使用原来的程序和库。需要后续再跑时使用全新的工作目录，例如：

```bash
mkdir -p /home/aaa/ORB-SLAM3_original0/results/original_baseline
mkdir /home/aaa/ORB-SLAM3_original0/results/original_baseline/new_run_01 && \
cd /home/aaa/ORB-SLAM3_original0/results/original_baseline/new_run_01 && \
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a \
  /home/aaa/ORB-SLAM3_original0/Examples/RGB-D/rgbd_tum \
  /home/aaa/ORB-SLAM3_original0/Vocabulary/ORBvoc.txt \
  /home/aaa/ORB-SLAM3_original0/Examples/RGB-D/TUM3.yaml \
  /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  /home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt
```

此示例保留历史 TUM3 的颜色配置；若需要严格比较颜色修正后的修改版，必须在
实验记录中单独说明原版使用哪个颜色配置。不要给原版设置指向 build_yolo11/lib 的
LD_LIBRARY_PATH，也不要把历史 B0 run_01 或旧失败目录作为新运行工作目录。

## 手动运行 TUM 并实时查看掩膜

在 Ubuntu 图形桌面的终端中执行（不使用 Xvfb，因为虚拟显示器不会出现在物理桌面）：

```bash
cd /home/aaa/ORB-SLAM3_original0
python3 tools/run_ablation.py --mode rgbd \
  --config configs/ablation/rgbd_tum3/yolo_only.yaml \
  --sequence /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  --associations /home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt \
  --output "$PWD/results/tum_live_$(date +%Y%m%d_%H%M%S)" \
  --show-masks
```

显示 `YOLO person - rgbd - mask (white=exclude)` 和 `YOLO person - rgbd - overlay` 两个窗口。
白色/红色区域均为实际传入特征提取器的剔除掩膜，开启后处理时显示膨胀后的结果。
替换为 `yolo_dilate.yaml` 可查看膨胀效果；加 `--max-frames 60` 可限制为短序列。
在任一窗口按 Q/Esc 会在当前帧跟踪完成后正常停止，导出已经处理的轨迹。
请使用 Q/Esc 结束；不依赖不同 HighGUI 后端行为不一致的窗口可见性查询。
窗口显示的是逐帧处理结果，CPU 推理不保证达到数据集原始帧率。

`--show-masks` 只修改本次结果目录内的 effective_config.yaml 为 `Yolo.ShowMask: 1`；
原始组配置保持默认 0。没有可用 DISPLAY/WAYLAND_DISPLAY 时明确报错。
仅显示掩膜时，Tracking 调用线程负责窗口；同时传入 `--viewer` 时，Tracking
只把最新图像和掩膜复制到有界邮箱，Viewer 线程统一刷新所有 OpenCV 窗口，
避免 GTK/HighGUI 多线程窗口冲突。此时窗口按各自最新结果刷新，不保证严格帧同步。
`Yolo.ShowMask` 不会自动启用保存或逐帧日志；正式测速省略 `--show-masks`。

### 同时显示 Current Frame、地图与掩膜

```bash
cd /home/aaa/ORB-SLAM3_original0
python3 tools/run_ablation.py --mode rgbd \
  --config configs/ablation/rgbd_tum3/yolo_only.yaml \
  --sequence /home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz \
  --associations /home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt \
  --output "$PWD/results/tum_live_viewer_$(date +%Y%m%d_%H%M%S)" \
  --show-masks --viewer
```

共四个窗口：`ORB-SLAM3: Current Frame`（跟踪特征）、`ORB-SLAM3: Map Viewer`
（相机/关键帧/稀疏地图）、二值掩膜和红色叠加图。地图初始为空，初始化后出现地图点。
地图窗口中可勾选 `Follow Camera` 跟随相机，用鼠标拖动/缩放查看地图。
在 Current Frame、掩膜或叠加图中按 Q/Esc，会结束当前 Track 后关闭显示并保存 TUM 轨迹；
组合预览模式下也可以点击地图菜单 `Stop`，或关闭地图窗口请求正常退出。
Viewer 负责窗口销毁，主线程等待其结束后导出轨迹，避免跨线程 HighGUI 调用。
这些显示开关不改变特征筛选规则；所有 GUI 绘制会影响耗时，正式性能对比不要启用。

运行包装器会同时把程序输出写入结果目录的 `run.log` 并实时回显到终端。每个 RGB-D/
Stereo 帧将输出一行 `[FRONTEND]`，其中 `raw_fast` 是掩膜和八叉树之前的 FAST 候选数，
`mask_rejected` 是 person 掩膜硬过滤数，`final_orb` 是最终带描述子的 ORB 特征数；
`matches` 为最终有效 MapPoint 对应数，`pose_inliers` 为局部地图位姿优化的内点数。
`keyframe=1` 表示该帧（包括初始化帧）实际插入了 KeyFrame。

## B1 / B2 YAML 开关（B2 配置骨架）

`DynamicDetector` 是 YOLO、B1 与 B2 的统一 YAML 读取入口；不会建立第二套解析器。
每个 RGB-D/双目消融配置以及 `Examples/RGB-D/TUM3.yaml` 都包含以下 B1/B2 字段。

```yaml
MaskFeatureReject.Enable: 1
B2.Enable: 0
B2.EvidenceOnly: 1
```

`MaskFeatureReject.Enable` 是 B1 的硬掩膜开关：为 `1` 时，YOLO person 掩膜传入
`Frame → ORBextractor` 并在 FAST 候选点后、八叉树前排除；为 `0` 时仍可进行 YOLO
推理/预览，但向特征提取器传递空掩膜。为保持旧的 YOLO 配置结果，缺失该键时默认是 `1`。

`B2Config` 还读取 person 候选点、跨帧关联、深度/三维几何和调试的全部字段。旧 YAML
缺失任意 B2 键时使用代码默认值，尤其 `B2.Enable=0`、`B2.EvidenceOnly=1`，因此不会
改变原有跟踪或位姿结果。启动日志会打印完整 `[B2 CONFIG]` 块，`semantic_timing.yaml`
也保存 B1/B2 启用状态。

当前 B2 是安全的配置与实验接口，绝不参与位姿优化：

- `include/B2MotionEvidence.h` 定义独立的 `B2CandidateFeature`、
  `B2PersonInstance` 与 `B2CandidateFrame`。它们不写入 `Frame::mvKeys`、
  `mvKeysUn` 或任意 MapPoint 索引容器；
- 当 `B2.Enable=1` 时，`Tracking` 创建 `mpB2Processor` 并保存当前帧的 B2
  自有 `frameId`、时间戳和（若可用）`Tcw` 快照。`B2MotionEvidence` 内部持有
  上一帧，不依赖会在 `Track()` 期间覆盖的 `Tracking::mLastFrame`；
- `YOLOSeg` 保留每个 person 的原始图像坐标 `mask`、`boundingBox`、`confidence`。
  B1 只使用所有原始实例的并集，并且仅在此并集上执行
  `MaskPostProcess.DilateRadius`；B2 保存未膨胀的单独实例掩膜。因而后续 B2 的
  `Candidate.ErodeRadius` 不会把 B1 膨胀进人物周边的背景错误当作 person 区域；
- RGB-D 已在 `GrabImageRGBD()` 中对每个 raw person mask 执行向内腐蚀，并以独立
  `ORBextractor` 提取至多 `Candidate.MaxFeaturesPerInstance` 个候选 ORB；候选带有
  原始 mask 内边界距离及配置范围内的原始/3×3 中值深度。腐蚀后小于一个 ORB 31×31
  描述子 patch（961 像素）的离散连通区域会被跳过，因此不会改变匹配、关键帧或地图
  决策；
- B2 绝不重读原始 TUM `CV_16U` 深度：它使用 `GrabImageRGBD()` 已依照
  `RGBD.DepthMapFactor` 转成 `CV_32F` 米制的同一份 `imDepth`。以候选像素为中心
  收集 3×3（关闭该选项时为单像素）中的有限非零值并取中值；该中值不在
  `Geometry.MinDepthMeters` 到 `Geometry.MaxDepthMeters` 范围内时，候选保留但标记
  `DEPTH_INVALID`，其 `depthMeters=-1`、`depthValid=false`；
- B2 当前明确只启用于 RGB-D / IMU-RGB-D。Stereo / IMU-Stereo 设置 `B2.Enable=1`
  时打印 `B2 RGB-D geometry is disabled for stereo input`，不会运行未验证的双目 B2
  几何路径。

### B2 person 跨帧实例关联

`B2MotionEvidence::Process()` 对上一 B2 帧和当前 B2 帧的 person 实例执行一对一贪心
关联。类别必须相同；候选对还须达到 `Association.MinInstanceIoU`、不超过
`Association.MaxCenterDistancePixels`，并且其 `0.5 * (IoU + 归一化中心接近度)` 分数达到
`Association.MinInstanceScore`。按分数从高到低选择且每个实例至多匹配一次。

已匹配的当前实例继承 `trackId`，记录 `previousInstanceId` 与 `associationScore`；未匹配的
当前实例创建新轨迹，未匹配的上一实例记录在 `LastEndedTrackIds()` 并结束。关联绝不把两个
person 的 `features` 合并，本帧未匹配实例不会计算跨帧三维残差。

对已关联的同一 `trackId` person，B2 对其候选 ORB 执行前向和反向最近邻 Hamming 匹配，
两向都应用 `Association.LoweRatio` 与 `Association.MaxHammingDistance`；默认要求互为最近邻
（`Association.UseMutualMatch=1`）。匹配结果写在当前 person 的 `matches` 中，输出
`candidateCount`、`forwardMatchCount`、`mutualMatchCount`、`matchedCount`、`depthValidMatchCount`。
其中 `candidateCount` 是上一帧向前查询的候选描述子数；`matchedCount` 是通过描述子/Lowe 和
配置的互检门槛、但尚未进行深度/几何检查的数目；`validGeometryCount` 才是最终具有可用三维
残差的数目。任一描述子匹配条件或两端深度有效性不满足时只记录无效原因，不构成动态判定。

### B2 相对位姿与三维残差

对 `MATCH_VALID` 的同一 person 匹配，且仅当两帧均保存了有效 `Tcw`、两端深度有效、
候选像素在各自图像内、相机内参有效时，`B2MotionEvidence::ComputeGeometryResiduals()` 计算：

```text
T_current_from_previous = current.Tcw * previous.Tcw.inverse()
P_predicted = T_current_from_previous * BackProject(previous_pixel, previous_depth)
raw_r3d = ||BackProject(current_pixel, current_depth) - P_predicted||
r2d = ||current_pixel - Project(P_predicted)||
```

`P_predicted.z > 0` 且投影落在当前图像范围内是额外的必经检查。每条
`B2CandidateMatch` 记录 `rawR3D`（米）、`normalizedR3D` 和 `r2D`（像素），并以
`GEOMETRY_VALID` / `GEOMETRY_INVALID` 区分结果。

### B2 背景 MAD 实例级诊断

启用 `B2.Geometry.Decision.Enable: 1` 且方法为 `background_mad` 时，B2 从连续两帧中同一
MapPoint 的 Tracking 内点、有效 RGB-D 深度、且位于两帧 raw person mask 外及
`Geometry.BackgroundExclusionDilateRadius` 圆形边界带外的观测计算背景
`r3d`。按 `Decision.DepthBinMeters` 深度分箱；每箱至少
`Decision.MinSamplesPerBin` 个样本，否则回退到当前帧全部背景样本。总背景样本少于
`Decision.MinBackgroundSamples` 时背景参考无效。每个参考均记录
`m_bg=median(r3d)` 和 `sigma_bg=max(1.4826*MAD, SigmaFloorMeters)`。

person 有效几何匹配以 `e=max(0, rawR3D-m_bg)/sigma_bg` 写入 `normalizedR3D`；所有 person
实例分别记录并判定，绝不会合并两个实例的候选或残差。三个比例的分母固定为：

- `match_ratio = matchedCount / candidateCount`；
- `valid_match_ratio = validGeometryCount / matchedCount`；
- `candidate_total_valid_ratio = validGeometryCount / candidateCount`，仅记录用于诊断，**不**作为
  判定门槛。

判定证据充分性的前置条件是 `matchedCount >= MinMatchedFeatures`、
`match_ratio >= MinMatchRatio`、`validGeometryCount >= MinValidFeatures`、
`valid_match_ratio >= MinValidMatchRatio`，以及两帧有效位姿和有效背景参考。默认值依次为
`30`、`0.15`、`10`、`0.40`。满足后：

- `E >= MovingThreshold`、高残差比例 `>= MinMovingFraction` 且
  `r3d median >= MinRawMovingMeters`：`GEOM_MOVING_CANDIDATE`；
- `E <= StaticThreshold` 且高残差比例 `<= MaxStaticFraction`：`GEOM_STATIC_CANDIDATE`；
- 其余：`GEOM_UNCERTAIN`；
- 证据不足、位姿不可靠或背景参考不足：`GEOM_UNRELIABLE`，并记录细分原因，例如
  `INSUFFICIENT_EVIDENCE`。

`r2d` 只记录、显示并用于失败分析，不参与上述静止/运动判定。默认的
`[B2 SUMMARY]` 每帧为**每个实例**输出一行：track/instance ID、candidate/matched/valid、
两个明确分母的比例、背景样本/median/sigma、R/E/P 与状态和原因；不再逐候选刷屏。
`B2.Debug.VerbosePerFeature: 1` 才会输出逐特征 `[B2 GEOMETRY]`，默认 `0`。
FrameDrawer 也为每个实例显示独立的 `ID/track/state/c/m/v/E` 行。即使得到 `*_CANDIDATE`，在 `B2.EvidenceOnly=1` 下它们
绝不修改特征使用、Tracking、位姿、关键帧、MapPoint 或地图。

- `B2.Enable: 1` 要求 `Yolo.Enable: 1`，否则明确报错；
- `B2.Enable: 1` 且 `B2.EvidenceOnly: 0` 明确报错：`B2 formal decision mode is not implemented yet`；
- `B2.Enable: 1` 且 `B2.EvidenceOnly: 1` 可用于后续接入证据记录，但当前不改变匹配、
  位姿、关键帧或地图决策。

`B2MotionEvidence::Process(const B2CandidateFrame&)` 是 EvidenceOnly 的只读边界：它复制
只包含 B2 数据的候选快照，在内部完成关联、残差和聚合，并返回 `B2Result`。该接口不接收
`Frame`、`MapPoint`、`LocalMapping` 或优化器对象；不会写入 `mCurrentFrame.mvpMapPoints`、
`mvbOutlier`、`N`、`mvKeys` 或 `mDescriptors`，也不会调用 `Optimizer::PoseOptimization`、
`ORBmatcher`、`NeedNewKeyFrame` 或 `CreateNewKeyFrame`。Tracking 只读当前帧以建立背景内点
快照，并只把返回结果用于日志和 FrameDrawer 显示。

### B2 CSV 与可复现实验元数据

当 `B2.Debug.SaveCsv: 1` 时，`B2.Debug.OutputDir` 必须指向已存在的实验结果父目录。程序会
创建唯一的 `b2_evidence_<timestamp>_<pid>/`，避免覆盖已有运行；其中包含：

- `b2_candidate_features.csv`：逐候选匹配的前/当前像素、Hamming 距离、两端深度、边界距离、
  `r3d`、背景 MAD 归一化残差、`r2d`、有效标志和原因码；
- `b2_instance_evidence.csv`：逐 person 实例的 track ID、candidate/matched/valid、
  `match_ratio`、`valid_match_ratio`、背景样本/median/sigma、R/E/P、位姿有效性、状态、
  判定原因和 B2 处理耗时；
- `b2_metadata.txt`：修改版 Git commit、YAML 路径与 SHA256、YOLO 模型路径与 SHA256、
  数据集名称和全部 B2 参数。

`B2.Debug.SaveCsv: 0` 时不会创建 B2 输出目录或打开 CSV 文件；`B2.Debug.SaveFeatureCsv: 0`
不写逐特征 CSV，`B2.Debug.SaveSummaryCsv: 0` 不写逐实例汇总 CSV。推荐将
`B2.Debug.OutputDir` 设置为每次实验自己的结果目录，而不是数据集目录。

### B2 Current Frame 诊断叠加

`B2.Debug.ShowOverlay: 1` 时，`FrameDrawer` 在 **Current Frame** 上叠加 B2 的独立候选
诊断点（不改变主 Frame 的 ORB 特征，也不参与任何 Tracking 决策）：蓝色为未匹配候选，灰色为
匹配无效或深度无效候选，绿色为有效低残差候选，红色为有效高残差候选，黄色为掩膜或深度边界候选。
窗口第二行的 `B2 candidates`、`B2 matched`、`B2 valid` 使用相同的
candidate/matched/valid 定义；它同时显示 `median r3D`、
`median normalized r3D`、`geometry evidence` 和 `B2 time`。红色的文字说明固定为
**High residual evidence**；它不是“Dynamic”标签，也不表示 B2 已对 SLAM 作出动态物体决策。

关闭 `B2.Debug.ShowOverlay` 时不会向 FrameDrawer 发送 B2 点或显示该诊断行。此开关仅影响可视化，
正式性能测试应关闭。

组合窗口回归测试命令（虚拟显示器仅供自动验证，实际看窗口不使用 Xvfb）：

```bash
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 tools/check_mask_preview.py --viewer
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 tools/check_mask_preview.py --viewer --focus-current-frame
```

本机组合显示实测通过（Xvfb + 软件 OpenGL，TUM 短序列）：

- 四窗口创建、叠加图按 Q：exit=0，相机轨迹 10 行，报告
  `results/yolo_preview_validation/20261019_131728_696598/preview_gui_test.json`。
- 四窗口创建、Current Frame 按 Q：exit=0，相机轨迹 13 行，报告
  `results/yolo_preview_validation/20261019_131754_046929/preview_gui_test.json`。
- `yolo_dilate` 四窗口自然结束：exit=0，相机轨迹 15 行、关键帧 4 行，
  推理和膨胀均 15 次，结果
  `results/yolo_preview_validation/combined_dilate_natural_20261019_1320/`。
- 原有仅掩膜模式回归通过：
  `results/yolo_preview_validation/20261019_132018_018060/preview_gui_test.json`。
- 无预览集成检查通过：
  `build_yolo11/validation/integration_20261019_131729_831833/results.json`。

以上只验证显示、退出及短序列轨迹生成，不代表完整序列精度验证或物理显示器实测。

实时预览已在本机 Xvfb 中验证：二值掩膜和叠加图窗口均成功创建，模拟 Q 后正常退出
（exit=0），导出 12 行相机轨迹、3 行关键帧轨迹。
测试报告：`results/yolo_preview_validation/20261019_130407_705256/preview_gui_test.json`。
关闭预览的原有集成检查也通过，报告位于
`build_yolo11/validation/integration_20261019_130257_686100/results.json`。
可用 `LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a python3 tools/check_mask_preview.py` 重复无屏幕检查；
用户实际观察窗口时直接在图形桌面执行运行命令，不使用 Xvfb。
