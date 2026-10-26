# 第10批：当前 B4 状态驱动的精细位姿策略

完整配置：`configs/frontend_modes/rgbd_tum3/mode3_fine_policy_validation.yaml`。
原第9批配置保留，不覆盖。

```yaml
DynamicFrontend.Mode: 3
TwoStage.CoarseFilter.Enable: 1
TwoStage.FinePolicy.Enable: 1
TwoStage.MapAdmission.Enable: 0
FeaturePolicy.EvidenceOnly: 0
B2.EvidenceOnly: 1
B3.EvidenceOnly: 1
B4.EvidenceOnly: 0
```

Mode=2继续禁止实际策略。Mode=3中，只有开启FinePolicy才允许B4.EvidenceOnly=0；
粗过滤对照继续要求B4.EvidenceOnly=1。地图准入开关仍拒绝开启，P应急补充不支持。
正式新配置的相机、ORB、B2六阈值、B4计数/TTL与第9批一致。

## 位置与时序

```text
整图ORB及语义标注
→ 原始初始跟踪分支 + 上一状态粗过滤
→ 粗位姿质量检查 → B2 → B3 → B4（每帧/track仅一次）
→ ApplyFineFeaturePolicy(pre_local_map)：使用当前状态
→ TrackLocalMap
    UpdateLocalMap
    SearchLocalPoints
    ApplyFineFeaturePolicy(pre_pose_optimization)
    VerifyFineOptimizerInputs
    原始PoseOptimization
    原始地图内点计数/成功门槛
→ 普通关键帧决策（失败显式禁止）
```

背景和当前S人物内部允许；当前P、D、人物边界、未知实例禁止。
第二次过滤处理局部地图搜索重新引入的匹配。
P→S在第一遍重新获得使用资格，但不会凭空恢复粗阶段的关联；局部地图搜索可以重新找到它。
S→P/D在第一遍立即清理旧关联。

禁止项仅清空当前Frame关联、清外点标记并置策略排除标志。
不删除/重排ORB关键点与描述子，不改变N，不调用SetBadFlag，不删除历史地图点。
精细策略覆盖本Frame粗阶段的策略元数据，因此帧末mvDynamicExcluded描述当前精细策略，不是永久删除标记。

## 支持不足与明确失败

第一遍匹配少不立即失败：先允许局部地图补充背景/S。
第二遍过滤后，静态地图支持不足则不调用精细优化，明确返回false。
优化后内点不足或位姿非有限也返回false。
门槛沿用原RGB-D TrackLocalMap：最近重定位窗口50；正常30；RECENTLY_LOST恢复且不在重定位窗口时11。
未添加新的可调阈值，也未改变B2/B4阈值。

失败输出 `FINE_TRACKING_INSUFFICIENT_STATIC_SUPPORT`，`fine_fallback=1`；
这里的fallback表示进入原始Track跟踪丢失/重定位状态路径，**不是恢复P/D匹配**。
NeedNewKeyFrame及CreateNewKeyFrame均在前置位置检查精细优化是否执行且成功，失败不能创建本帧普通关键帧或进入其深度地图点创建代码。
已有LocalMapping队列继续正常工作，不因本帧失败而全局暂停/删除历史点。
RGB-D首次初始化仍沿用原始直接建立初始地图的路径，尚无粗/精细跟踪可运行；日志标记未执行精细策略。

## 日志及作用范围

`fine_feature_policy_*.csv`和终端`[FINE POLICY]`：

- 阶段为pre_local_map、pre_pose_optimization、result；常规成功帧各一行；初始化没有这三行。
- fine_matches_before/after为该次过滤输入/输出匹配数。
- fine_reject_P/D/boundary/unknown只计算非空匹配，before−各reject之和=after。
- fine_inliers在过滤阶段为−1，result为实际精细内点数；未执行优化而失败时为0。
- P_at_optimizer、D_at_optimizer、forbidden_at_optimizer须为0；优化入口另有独立检查，漏过滤会明确报错。
- fine_fallback、fallback_reason、required_static_support、optimizer_executed、keyframe_blocked用于解释失败。

B2/B3保持NONE_EVIDENCE_ONLY并继续检查不修改位姿、关联、外点和描述子。
B4日志在Track结束后输出：正常执行策略写FINE_USE_NORMAL/FINE_EXCLUDE_PENDING/FINE_EXCLUDE_DYNAMIC，
未运行写NONE_FINE_NOT_RUN，TTL丢失实例写NONE_NO_CURRENT_INSTANCE。
FINE_表示已应用的精细使用策略，不保证该帧跟踪最终成功，也不表示地图准入已实施。

旧feature_policy_summary CSV仍在B4更新后、精细过滤前生成，actual_excluded_count仍对应粗阶段快照；
本批精细结果以fine_feature_policy CSV为准。
现有`[FRONTEND] person_mask_ratio`仍从Frame的B1提取掩膜统计；Mode=3传入空提取掩膜，
该旧字段可能为0，不能据此判断YOLO无人检出。精细策略使用独立的原始实例标注，不读取该日志字段。

**地图准入仍关闭。成功关键帧的原始深度建点逻辑尚未逐特征接入策略，P/D仍可能被建为新地图点。**
因此本批只保证最终精细优化不使用P/D，以及精细失败帧不提交普通关键帧，不声称已经完成动态地图闭环。
成功关键帧的建点步骤还可能在优化之后重新填入P/D关联，因此不能用返回时Frame的非空指针数反推优化输入；
应查看pre_pose_optimization及入口断言。此问题留给后续地图准入批次，不在本批偷偷改变建点策略。

## 验证与复现

隔离build_yolo11，使用-j2，保留原版可执行文件/库及旧配置/结果，不自动提交Git。
新增fine_feature_policy_test覆盖当前/历史状态区别、两次过滤、局部搜索重引入、最终入口检查、
S→P/P→S、新P/边界/未知、索引/描述子不变、支持不足、关键帧门控、空/错误尺寸。
tools/audit_fine_feature_policy.py核查源码范围、调用时序、共享ID、B4一次更新与冻结，以及最终P/D输入为0。
先验收全部100帧片段，通过后再跑完整序列：

```bash
cd /home/aaa/ORB-SLAM3_original0
python3 tools/run_frontend_stage_validation.py \
  --stage batch10_manual_validation --short-only \
  --mode-config 3=configs/frontend_modes/rgbd_tum3/mode3_coarse_filter_validation.yaml \
  --mode-config fine3=configs/frontend_modes/rgbd_tum3/mode3_fine_policy_validation.yaml \
  --require-main-person
python3 tools/audit_fine_feature_policy.py <新短片段结果目录>
```

完整运行使用相同参数、去掉--short-only，加入--short-reference <已验收短片段目录>。
Viewer关闭，避免图形阻塞。运行目录自动带时间戳，保存配置、commit、命令、哈希、日志、CSV和两条轨迹。
本批结果在完成后补充；编译通过不等于精度提升，真实失败/重定位与GUI需分别标明是否实测。

### 已完成前置验收

构建中断遗留了8个0字节对象，恢复构建首次链接报未定义引用；已保留完整日志与空对象清单，
仅在隔离build_yolo11中强制重编译这些对象，未更改算法源码来规避链接错误，也未删除旧日志。
最终构建使用-j2，RGB-D/Stereo/Monocular示例、runner与测试目标均编译通过。
原版二进制/库SHA256检查通过，未被覆盖。

9项单元测试、48项配置合法性检查通过。四个100帧片段（粗过滤对照/精细过滤 × sitting/walking）
全部100帧OK，无RECENTLY_LOST/LOST，源码与CSV审计通过。
精细优化前P=D=0；第二遍分别排除sitting的2687个P匹配、4870个边界匹配，
walking的9540个P匹配、4106个边界匹配、9个未知匹配。
walking第一遍另排除54个匹配，sitting第一遍无额外排除；短片段未出现D匹配，D路径由单元测试覆盖。
两种模式的B4均保持每frame/track一次更新，不可靠证据冻结状态和有效计数。

位置：

- `results/frontend_stage_validation/batch10_fine_policy_build_01/`
- `results/frontend_stage_validation/config_checks_20261023_092622/`
- `results/frontend_stage_validation/batch10_fine_policy_short_20261023_092645/`

精细支持不足和关键帧门控已通过人工输入单元测试，暂未在真实序列中自然触发精细失败，
不能将正常序列通过写成真实困难场景的重定位恢复验证成功。

### 完整序列验收

全部结果位于：
`/home/aaa/ORB-SLAM3_original0/results/frontend_stage_validation/batch10_fine_policy_full_20261023_092853`。
每个子目录保存完整命令、commit、配置快照及哈希、runtime哈希、run.log、B2/B3/B4 CSV、
粗质量与粗过滤/策略CSV、两条轨迹、退出码与validation.json；精细模式另有fine_feature_policy CSV。
runner中的`3`为第9批粗过滤对照标签，`fine3`为第10批精细模式标签，两者YAML的DynamicFrontend.Mode都为整数3。

| 配置 | 序列 | 相机轨迹/输入 | 关键帧轨迹 | 第二次排除P | 第二次排除D | 精细失败 | RECENTLY_LOST/LOST |
|---|---|---:|---:|---:|---:|---:|---:|
| 粗过滤对照 | sitting_static | 680/680 | 17 | 不应用 | 不应用 | 不应用 | 0/0 |
| 粗过滤对照 | walking_xyz | 827/827 | 225 | 不应用 | 不应用 | 不应用 | 0/0 |
| 当前状态精细策略 | sitting_static | 680/680 | 30 | 2678 | 0 | 0 | 0/0 |
| 当前状态精细策略 | walking_xyz | 827/827 | 265 | 21732 | 10960 | 0 | 0/0 |

所有退出码均为0。精细模式的常规优化分别679/826次，每次均验证P_at_optimizer=D_at_optimizer=0；
初始化帧无精细优化。sitting第一遍排除0，第二遍另排除28504个边界匹配；
walking第一遍排除60，第二遍另排除15346个边界匹配及23个未知匹配。
这是匹配次数总和，不是独立物理特征数；result行重复第二遍输入统计，汇总只统计实际过滤阶段，不重复累加result。

精细模式的B3/B4实例帧统计：sitting的MOVING=0、D=0，S=1330、P=30；
walking的MOVING=33、D=349，S=323、P=348。
这些不是运动真值，也不能用来宣称运动分类精度或ATE改善。

`fine_policy_audit.json`对短/完整共8个运行通过：

- 验证两次过滤顺序、优化入口全类禁止关联为0、所有必须的日志字段与计数等式；
- 验证B4每帧/track一次更新、共享ID、无证据冻结，以及applied字段作用范围；
- 核对B2接口/算法、B3/B4规则、重定位、粗跟踪与后端未改；
- 确认普通关键帧与其深度点创建之前有失败保护。

`fine_policy_run_summary.csv`包含全部短/完整运行汇总；`extraction_policy_audit.json`逐帧核对两模式的
raw_fast/final_orb数量相等、B1 mask_rejected均为0，证明过滤位于使用阶段而非FAST阶段。
这不是描述子逐字节或轨迹精度等价的证明。

未出现自然精细失败，失败不恢复P/D与关键帧保护目前由单元测试及源码审计覆盖；
真实困难场景失跟/重定位、GUI和新模式ATE/RPE尚未验证。
原版二进制/库哈希保持不变，旧配置/结果保留，未提交Git或切换分支。
本批源文件最终归档为build日志目录中的implemented_sources.tar.gz，另有修改前快照和差异补丁。
