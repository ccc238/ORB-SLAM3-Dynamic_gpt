# B2 三区域阈值配置

`DynamicDetector` 使用现有 YAML 读取路径解析六项阈值，
`B2MotionEvidence::ComputeInstanceDecisions()` 消费这些生效值。
`Examples/RGB-D/TUM3.yaml` 已显式填写平衡方案：

```yaml
B2.Geometry.RawMovingThreshold: 0.020
B2.Geometry.NormMovingThreshold: 0.40
B2.Geometry.HighRatioMovingThreshold: 0.10
B2.Geometry.RawStaticThreshold: 0.018
B2.Geometry.NormStaticThreshold: 0.10
B2.Geometry.HighRatioStaticThreshold: 0.06
```

通过原有可靠性检查后，运动要求 R/E/P 均不小于运动阈值；静止要求
R/E/P 均不大于静态阈值；其余为 UNCERTAIN。不可靠检查、B3/B4 规则
及 EvidenceOnly 边界不变。新字段配置需满足三个静态阈值均严格小于
对应运动阈值；建议始终完整填写六项，部分配置只有在回退值仍满足
合法性检查时才可使用。

可靠性门控依次检查：前后帧位姿、候选数（沿用原实现至少 1 个）、
`MinMatchedFeatures`、`MinMatchRatio`、`MinValidFeatures`、
`MinValidMatchRatio`、背景参考数量/有效性与归一化残差完整性。
任意失败都保持 `GEOM_UNRELIABLE`，记录对应原因码。只有全部通过，
才在独立分支中计算 `moving` / `stationary`，按运动、静止、不确定
的顺序赋值；日志输出保留在分支外，失败实例仍完整记录。

## 旧 YAML 兼容

新字段优先，缺失时回退如下：

| 新字段 | 旧字段 | 无旧字段时默认 |
|---|---|---|
| RawMovingThreshold | Decision.MinRawMovingMeters | 0.02 |
| NormMovingThreshold | Decision.MovingThreshold | 3.5 |
| HighRatioMovingThreshold | Decision.MinMovingFraction | 0.60 |
| RawStaticThreshold | 无 | +inf（不限制 raw） |
| NormStaticThreshold | Decision.StaticThreshold | 2.0 |
| HighRatioStaticThreshold | Decision.MaxStaticFraction | 0.20 |

上述名称前缀均为 `B2.Geometry.`。旧规则没有 raw 静态上限，使用
正无穷回退可保持旧规则的结果。新 YAML 显式输入必须是有限数值。

## 保持 high_ratio 的含义

`P = count(e_i >= point_cutoff) / valid_count`，点级 cutoff 继续读取
`B2.Geometry.Decision.MovingThreshold`（默认 3.5）。这与离线标定的原始
CSV 一致。新增 `NormMovingThreshold=0.40` 只作用于实例中位数 E，
不会把点级 cutoff 一并降到 0.40。Viewer 高残差着色也使用此点级 cutoff。

启动配置日志和 `b2_metadata.txt` 均记录六项生效值及点级 cutoff。
存在新字段时启动日志说明其优先级。已有实验 CSV/轨迹不受配置迁移影响。

可用 `build_yolo11/bin/yolo_seg_smoke --config-only <yaml>` 检查配置。
配置工具遵循 YOLO 开关；加载模型前验证阈值，无需运行完整 SLAM。

## 本次验证

使用 `cmake --build build_yolo11 --target rgbd_tum yolo_seg_smoke -- -j2`
完成编译。通过配置工具核验：

- 六个新字段全部缺失：RD=.02、ED=3.5、PD=.60、RS=inf、ES=2、PS=.20。
- 旧字段自定义：回退值保持旧 YAML 的设置，点级 cutoff 也保持旧值。
- 新平衡配置：读取六个指定值，点级 cutoff 仍为 3.5。
- 静态/动态区重叠：程序退出码 2，给出阈值分离错误。
- 实际 `Examples/RGB-D/TUM3.yaml`：配置解析及本地模型加载成功。

本次未重新运行数据集；编译与配置验证不代表新参数的在线精度验证。
