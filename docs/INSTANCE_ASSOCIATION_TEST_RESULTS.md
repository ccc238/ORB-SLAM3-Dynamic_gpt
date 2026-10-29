# 本轮实例身份关联：实际验证记录

分支 `snapshot_clean`，HEAD `47ba0c174edfc4cfd7b6399b0f5b80c6b5b3b4b9`。未提交、推送、合并或丢弃已有改动。

## 构建

独立目录 `/home/aaa/ORB-SLAM3_original0/build_instance_association`，所有可执行程序/库在该目录 bin/lib。完整 CMake 构建退出 0，包括 mono、stereo、RGB-D 和 old examples 的共享接口编译；未运行单目/双目数据集。

使用 -j2。Optimizer.cc 单独采用 GCC 编译期垃圾回收参数降低峰值内存，之后仍 -j2 构建。未修改 Optimizer.cc、ORBmatcher.cc、MapPoint.cc、后端 BA 或旧相机 YAML。

成功构建日志：`build_instance_association/build_consistent_final.log`；相关低内存构建日志 `optimizer_rebuild_lowmem.log`。后续修改只涉及 Python 评测工具、说明和新配置注释，不改变已经测试的 C++ 或 YAML 参数。每次运行的 effective.yaml 与哈希是运行时原件。

## 单元、集成与配置

`python3 tools/validate_instance_association.py --output build_instance_association/validation_03 --short-sequences --full-sequences`

15 项测试全部退出 0：instance_association_test、identity_state_integration_test、b4_evidence_expiry_test、b4_state_machine_test、instance_track_lifecycle_test、frontend_diagnostics_test、keyframe_decision_guard_test、map_admission_test、fine_feature_policy_test、coarse_feature_filter_test、coarse_pose_quality_test、feature_policy_test、feature_semantic_test、frontend_mask_test、b2_candidate_test。

覆盖合成单人/相机平移 LK、匈牙利全局解、两目标竞争/合并/分裂、短时遮挡重现、深度不一致新人、TTL 边界与跳帧预过滤、无效深度/纹理、重复/倒退/NaN 时间、ID 不复用，以及 B2/B4/粗精策略/KeyFrame 准入的受控继承。

四个新配置均合法；缺失关联配置默认 A3；负数/过大/小数模式、零代价、NaN margin、真实匹配比未匹配总成本更贵、证据项不足等非法配置按预期返回 2。不是将非法配置退出 2 误算失败。

真实 System 集成：sitting 前 3 帧 → ResetActiveMap → 从原采集时间重新输入 3 帧 → Reset → 再输入 3 帧。退出 0；日志中三个 sequence_id，身份 ID 不跨序列复用，新序列首帧两个权限均关闭。

Python 独立评测接口：`python3 tools/test_instance_identity_evaluation.py` 通过合成手工真值、ID 切换/碎片/错误 S 使用/恢复时间/使用比计算、缺失真值拒绝及防覆盖检查。该合成测试不是 TUM 身份正确率。

## 真实序列

A0～A3 × sitting_static/walking_xyz，各 100 帧的八次运行全部退出 0，均为 100 个有效轨迹位姿、零 LOST/RECENTLY_LOST。每次精细策略审计：P、D、forbidden_at_optimizer 全部为 0。地图最终审计：forbidden_observations=0，unaligned_keyframes=0。

A3 完整结果（只统计可见实例帧；不将 TTL 丢失显示行混入实例状态比例）：

| 序列 | 图像/相机轨迹位姿 | 可见实例帧 | 连续确认 | 重关联 | 歧义拒绝 | 系统 ID 数 | 可见 S/P/D |
|---|---|---|---|---|---|---|---|
| sitting_static | 680/680 | 1360 | 1358 | 0 | 0 | 2 | 1323/37/0 |
| walking_xyz | 827/827 | 1020 | 963 | 18 | 25 | 39 | 227/610/183 |

两个完整序列均退出 0、全帧 Tracking=OK；两条轨迹文件非空，B2/B3/B4 CSV、诊断与关联 CSV 均保存。

walking 的 57 个不允许状态继承实例帧 = 14 NEW + 25 AMBIGUOUS + 18 REACQUIRED：

- 粗阶段正常授权数 0；实际 coarse S 特征输入 0。
- 实际 coarse 所有特征输入也为 0（只针对这 57 个实例帧，不推广至所有回退场景）。
- 精细优化实际输入 0；地图准入授权 0。
- 几何配对禁止 57，B2 几何可靠均为 false；没有重复 B4 frame/track 更新。
- 相机轨迹 827 行，不是通过每帧新建 ID 来获得该权限结果：963/1020 个可见实例帧是连续确认。但临时歧义 ID 确实增加碎片，39 个系统 ID 不能被当成 39 个人，也不能当成人工真值 ID switch。

完整序列关联耗时均值（有检测实例的帧，含现有日志运行条件，不是正式测速）：sitting 8.93 ms、walking 7.72 ms。

另执行 `python3 tools/validate_instance_association.py --output build_instance_association/compatibility_01 --compatibility`：再次通过单元及配置检查；ORIGINAL、B1、旧 Mode2 evidence 配置各运行 sitting 前 3 帧成功。ORIGINAL 的模型路径故意设为不存在，仍成功且未加载 YOLO。这里是启动/接口兼容验证，不是旧轨迹逐位一致或精度等价证明。

## 输出位置

全部新输出在 `/home/aaa/ORB-SLAM3_original0/build_instance_association/`：

- `validation_03/summary.json`：单元、配置、System reset、短测、完整序列的退出状态和耗时。
- `validation_03/sequence_summary.json`：自动审计统计，states 字段含 TTL 显示行，不能与可见状态表混用。
- `validation_03/A3_controlled_sitting_static_full/` 与 `A3_controlled_walking_xyz_full/`：完整日志、命令、配置快照/哈希、commit、轨迹和 B2/B3/B4/关联/策略 CSV。
- `measured_audit_03/run_audit_summary.csv`：十次真实序列的可见状态、权限与实际特征使用审计。
- `measured_audit_03/system_track_continuity.csv`：系统轨迹可见长度、间隔、歧义/重关联次数，不是人工身份正确率。
- `measured_audit_03/source_snapshot.tar.gz`、`source_sha256.json`、`worktree.patch`：未提交源码快照及校验，包含新源文件/工具/配置，不包含模型和数据集。
- `annotation_A3_walking_full_01/manual_annotation_template.csv`、`annotation_A3_sitting_full_01/manual_annotation_template.csv`：完整序列独立人工标注模板，未伪造标签。
- `compatibility_01/`：旧模式启动及缺模型基线验证日志。

## 失败与修正记录（保留未覆盖）

1. 最初独立 g++ 探针未链接 ONNX Runtime，报 undefined reference to OrtGetApiBase，见 association_standalone_build.log。正式 CMake 链接正确，后续关联测试通过。
2. validation_01 在构建尚未结束时启动，前五项通过后因 frontend_diagnostics_test 还不存在而停止。不是缺依赖安装问题。
3. validation_02 首次 A0 sitting 第 0 帧后退出 -11。GDB 在沙箱内 ptrace 被拒，申请权限后获得 FrameDrawer::Update → cv::Mat::empty/copyTo 堆栈，保存在 crash_diagnostic_01/。
4. 当时头文件在并行编译期间更新，Tracking/TrackingDiagnostics 对象时间与布局不一致。统一重编译相关对象时还暴露旧 TrackingDiagnostics 对象缺失 RecordIdentityOptimizerInputs 符号（build_final.log）。强制重编译 Tracking、TrackingDiagnostics、LocalMapping、FrameDrawer 后完整构建成功，validation_03 全部通过。不能复用 validation_02 的半成品轨迹作为样本。
5. A0 新增元数据的 -1 哨兵曾使第 0 帧误标相邻配对，已加 `lastMatchedFrame>=0`，并新增四模式首帧禁止配对断言。最终运行与测试已包含此修正。

以上日志全部保留。没有覆盖旧实验，未下载模型、安装依赖或套用外部补丁。

## 未完成验证及下一步

尚无独立人体身份/运动真值，所以未提供真实 ID switch、碎片真值、错误 S 继承率、动态泄漏率或静态保留正确率；模板生成不算人工标注完成。也未做 A0/A1/A2 全序列多次重复、精度/ATE/RPE 对比、GUI 人工交互、长时间完全丢失导致 Track 内建图的专项压力测试。

当前只有平移预测、无外观辅助或 ReID。相似目标在同位置连续替换可能无法仅凭空间/深度区分。持续合并会创建临时 ID，必须审查连续性成本。旧粗跟踪应急回退仍可恢复 P 关联；本次不可信实例帧没有观察到这种实际使用，不代表所有未来路径均零使用。

实际优化器输入计数仅覆盖正常初始跟踪及 TrackLocalMap，未采集未修改的 Relocalization 内部优化。不要将它报告为覆盖所有重定位/应急路径的完整安全证明。

建议先对两个完整序列人工标注交叉/遮挡/换人/合并片段；使用四组相同真值比较身份连续性、错误 S 的实际优化器输入、重关联恢复及静态重用，再开展重复完整序列和 SE(3) ATE/RPE 消融。不得从当前通过结果预先宣称定位精度提升或论文结论。
