**PNP7 数据真实语义与质量评估 · 2026-09-09**

结论：采集层已经保留了有价值的机器人反馈、应用层动作目标、双路图像和部分诊断信息；但目前还不能把导出的数据称为“语义契约完整、时间连续性有保证、自动质检可信”的训练集。最需要处理的是两条训练导出路径的动作定义不同、跨暂停计算动作、指令记录层级不明确、质量检查可放过明显坏数据。

本次依据用户提供的《0A-地基A-机器人数据语义与格式》评估实际代码。文档中的案例、建议与面试话术作为评估框架，不作为需要执行的指令，也不直接当作 PNP7 的事实。

范围：当前工作区（包含用户原有未提交修改），基准 commit `fbc01498e4d5d45102462aac63e8532e2ea918ea`；另追踪同级 `training/convert_to_lerobot.py` 和当前 `inference/`。没有在本机 `vla_data_collect` 下发现真实 episode.csv、teleop.csv 或 LeRobot 数据文件。因此本报告区分“代码确认”“合成复现”“尚待实测”；README 中的旧测量以及推理文件中提到的 30 条示范，不作为本次实际验收统计。机器人上的部署版本和 checkpoint 对应的数据转换版本尚未核实。

审计源文件的 SHA-256 已记录在 [sources.json](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/docs/data_semantics_audit_2026-09-09_sources.json)，以免仅凭 commit 遗漏工作区修改。

**1. 到底有几层数据、几种 action**

```text
PNP7 被动主臂 + 脚踏板
  → 主臂解缠绕、死区、相对映射
  → PNP7 SafetyChain（限幅、滤波、速度/加速度约束）
  → q_target（此处记录）
  → libfranka 再滤波、限速
  → Franka 执行 → q_robot / O_T_EE 实测反馈

teleop.csv + JPEG / 相机索引 + 可选事件流
  → build_episode.py：外部相机锚定，最近邻匹配，去除松踏板行
  → episode.csv
      ├─ collect/export_lerobot.py：8 维关节绝对目标
      └─ ../training/convert_to_lerobot.py：7 维实测末端转移增量
```

这两种动作表示不能只用同一个 `action` 名称解释，也不能共用同一套归一化、执行器与维度契约。当前推理代码的 `(10, 7)` 输出和末端增量执行符合第二条路径；仅凭代码仍不足以证明某个已经训练的 checkpoint 实际使用了哪版转换器。

**2. 原始/对齐层字段的真实含义**

| 字段 | 实际含义与单位 | 重要限定 |
|---|---|---|
| `q_robot0..6` | Franka J1..J7 实测关节角，rad | 来自 `robot_state.q`，不是主臂角或指令回显 |
| `dq_robot0..6` | 实测关节角速度，rad/s | 不等同于派生动作 `dq_action` |
| `tau_robot0..6` | 连杆侧关节力矩传感器信号，N·m | 不是外力矩或电机电流 |
| `q_target` → `q_command` | PNP7 安全链输出的绝对关节位置目标，rad | 位于 libfranka 后处理之前；不是“已确认送达/执行的最终指令” |
| `lead_delta` → `q_master` | 本次进入 TELEOP 时的主臂参考点到当前主臂位置的差，rad | 已解缠绕、经过死区；尚未乘 sign/scale；既不是原始编码器也不是主臂绝对角度；重接管重置参考点，暂停时写零 |
| `O_T_EE0..15` | 基座 O 系中的实测末端 EE 位姿，4×4 列主序矩阵；平移 m，旋转无量纲 | 平移在数组 12/13/14；没有保存相应工具/刚度参考系配置快照 |
| `O_F_ext0..5` | 作用在刚度框架 K、用基座 O 系表达的估计外力/外力矩，N/N·m | 不是独立六维力传感器真值，不能把全零直接解释为绝对无接触 |
| `gripper_width` | 异步读取并缓存的实测夹爪宽度，m | 数值越大越开；每行复制缓存，没有样本时间戳/年龄；代码注释说状态流约 5 Hz，本次未实测 |
| `gripper_target` → `gripper_command` | 首次 TELEOP 前是连接时实测宽度的初始化值；接管后是扳机映射出的期望开度，m | 常用配置为二值 0 或 open_width；配置 gripper_open_width=0 表示采用设备 max_width，并非要求闭合；0 目标会触发 grasp，物体存在时实测宽度不必为零；尚未证明本行对应一次实际发送 |
| `gripper_ticks` → `gripper_master_ticks` | 主臂第 8 个伺服器的连续 ticks，4096 ticks/转 | 记录点已在解缠绕和死区处理之后，不能称完全原始 |
| `state` | PNP7 状态机：0 READY / 1 TELEOP / 2 PAUSED | 不是 Franka robot_mode |
| `deadman` | 本次日志采样时脚踏板是否按下 | 按下不必然等于 TELEOP：主臂过期、结束减速时都可能按着但暂停 |
| `teleop.t_ns` | FCI 回调进入后的主机 CLOCK_MONOTONIC，ns | 不是机器人设备采样时间，也不是绝对 UTC |
| `dt_s` / `lead_seq` | 控制器 callback period（s）/ 主臂有效快照序号 | 比单纯数日志行更能判断控制周期与重复主臂样本；对齐层未继续保留 |
| `episode.t_ns` | 外部相机主机接收时间，ns | 最近邻匹配，可选到锚点之后的状态；skew 只保存绝对值 |
| `rgb_external` / `rgb_wrist` | JPEG 文件路径；采集默认 640×480、30 FPS、JPEG quality 90 | 采集内存是 BGR；两条导出都显式转 RGB；只采集 color，没有深度/IMU |
| `event_t_start_us/end_us` | 上一个原始锚点到当前锚点在事件相机时钟中的窗口，μs | 不是全局时钟；需 events_meta.json 的映射和 events.hdf5；两条所查训练导出均未消费事件流 |

证据：[回调记录点](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/src/pnp7_teleop.cpp:1594)、[主臂差值定义](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/src/pnp7_teleop.cpp:866)、[字段转换](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/collect/build_episode.py:151)、[相机采集点](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/collect/record_cameras.py:100)。SDK 的单位、位姿排列和外力参考系按 [libfranka 0.21.3 RobotState 定义](https://github.com/frankarobotics/libfranka/blob/0.21.3/include/franka/robot_state.h) 核对。

**3. 按文档的“action 六问”回答**

| 问题 | collect/export_lerobot.py | training/convert_to_lerobot.py |
|---|---|---|
| 指令还是反馈？ | 应用层关节目标 + 夹爪期望目标 | 前 6 维来自未来实测末端变化；最后一维来自下一行夹爪指令 |
| 绝对还是增量？ | 8 维全部按绝对目标解释 | 平移/旋转为增量；夹爪为绝对开合 |
| 关节还是末端？ | 7 关节 + 夹爪 | 3 平移 + 3 旋转增量 + 夹爪 |
| 单步还是 chunk？ | 文件逐帧存单步 | 文件逐帧存单步；本地配置/推理涉及 10 步 chunk，训练尾部 padding 需查实际 dataloader |
| 夹爪是什么？ | 单位 m，通常 0 或约 0.08；缺失值错误地转成 0 | 二值 0=关、1=开；state 也用指令二值，不是实测宽度 |
| 一个动作管多久？ | 从约 1 kHz 目标流抽出的相机时刻样本，并没有记录“该目标保持 33 ms”的事实 | 默认按 30 FPS 写出，但计算时没有检查源行是否相隔 1/30 s |

关节导出的定义：

```text
state[k]  = [q_robot[k, 0:7], gripper_width[k]]
action[k] = [q_command[k, 0:7], gripper_command[k]]
```

`episode.csv` 还派生了 `dq_action[k] = q_command[k+1] - q_robot[k]`，单位 rad，不是 rad/s，也不是 `q_command[k]-q_robot[k]`；末行没有下一帧，代码退化为本行 command-state。它并没有被上述两个 exporter 用作训练动作。

末端导出的定义，设 `g[k] = 1(gripper_command[k] > 0.04 m)`（默认阈值）：

```text
state[k]  = [p_actual[k], first_two_columns(R_actual[k]), g[k]]  # 10 维
action[k] = [p_actual[k+1]-p_actual[k],
             rpy(R_actual[k].T @ R_actual[k+1]), g[k+1]]         # 7 维
```

平移增量用基座系；相对旋转使用当前末端的局部参考系，执行时右乘 `R_next = R_current @ R_delta`。这是一种可以使用的混合参考系约定，但必须写清。此处 `drx/dry/drz` 是小相对旋转的 RPY 参数，不是世界系欧拉角直接相减，也不是轴角旋转向量。state 的 6D 旋转规避了绝对欧拉角 ±π 跳变，这是正确的设计。

证据：[关节 exporter](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/collect/export_lerobot.py:82)、[末端 converter](/Users/andyliu/workspace/vla_data_collect/training/convert_to_lerobot.py:76)、[当前推理执行](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/inference/pnp7_policy.py:763)。

**4. 按影响排序的质量问题**

**P1：训练目标已经发生语义变化，需要按控制接口验收。** 末端 converter 完全不使用 q_command，而是从两帧 O_T_EE 反推实际运动。即使关节指令变化而机械臂暂时没有移动，末端平移动作也可能是零。它学到的是这套遥操作控制器作用下的实际转移，并不直接等价于人的原始指令意图。

这不意味着“使用反馈差分必然训练失败”：把实际未来轨迹定义为期望轨迹也是一种可行训练目标。风险在于采集时的跟踪延迟、接触响应、滤波与推理控制器不同；需要用相同部署接口回放并测量误差，才能判断反馈增量标签是否合适。应把两种定义分别命名为 joint_absolute_setpoint 和 eef_measured_transition，绑定转换版本及归一化统计；不能只写 action_is_delta=true。

**P1：暂停边界会被当成连续动作。** build_episode 先删除松踏板行，再直接使用剩余表的“下一行”算 delta；末端 converter 同样相邻差分，甚至没有读取 t_ns、state、deadman 或 segment_id。暂停一秒之后的实测变化，会被写成下一步 30 Hz 动作。关节 exporter 虽提供分段，但只认大于 200 ms 的缺口，默认不拆 episode；150 ms 的暂停会漏检，chunk 是否跨段取决于外部 dataloader。应从原始状态机转换生成 segment_id，在任何差分、chunk、重采样之前切分；段末 action 应显式标无效或采用有记录的 padding 策略。

补充复核：接缝动作不一定大。暂停期间通常先减速再保持，跨接缝位姿差可以极小乃至为零。新增合成用例中，相隔 1 秒、位姿完全相同的两帧仍产生一个六维全零动作。此前 5 cm 示例只是证明转换器不检查时间，并非对真实暂停运动幅度的估计。因此“时间不连续”本身才是缺陷，不能依赖跳变阈值发现它；也不能只从已经筛过的 episode.state 切段，因为中间 PAUSED 行可能已经被删除。

**P1：夹爪持久状态复制是必须单独验收的退化策略。** state 的 g[k] 与 action 的 g[k+1] 只有切换时不同。使用真实 load_episode() 和另造的 585 行合成输入（两次开合切换），得到 584 个单步标签：直接复制 state 的准确率 99.6575%、L1=0.00342466，但切换帧准确率为 0%。主要 live 路径会把自身 command_open 再输入模型；若模型各执行步始终复制该值，确实会形成开着一直开、关着一直关的固定点。输入在推理时“可得”不能排除这种退化。

这些数字仅刻画本次合成数据的稀疏事件比例，不是实际 checkpoint 的表现，也不等于 10 步 chunk 的实际训练损失。验收必须报告开/关事件的召回、误触发、时刻偏差、切换附近窗口指标，以及按实际 chunk/执行步数计算的复制基线；全局逐帧准确率不足以判断会不会抓取。

**P1：非 TELEOP 行仍可进入示范。** 筛选仅要求 deadman=1。C++ 实际 enable 还要求主臂新鲜和未到结束时间。因此 watchdog 暂停或结束减速期间，脚仍踩着的数据会留下。应基于一致的 enable/state 快照过滤，并记录暂停原因与主臂年龄；正常示范中的有意静止仍应保留，不能用“动没动”替代有效性判断。

**P1：PASS 尚不能作为训练集准入。** 现有 validator 没有 finite 检查、图片解码、时间戳单调/间隔检查、实质的相机 skew 门槛、跟踪误差门槛和任务成功判定；“实测与指令不完全相同”只能捕获一种回显错误，无法证明传感器健康。GUI 会保存 build_ok=false，但后续 validator 仍可能给 PASS；正常 end_episode 即使校验失败也返回 kept=true，故 kept 是保存动作而非验收结果。两个 exporter 都按 episode.csv 存在与否纳入，没有统一读取 quality verdict/failure.json。

已实际复现：单个 q_command=NaN、360 张完全无法解码的非空“JPEG”、wrist skew=500 ms，分别都能通过现有 validator。应先修这些门槛，再使用自动 PASS 作准入；缺少任务成功验证时，不能称“fit to train”已得到充分证明。

证据：[筛选与差分](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/collect/build_episode.py:117)、[校验器](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/collect/validate_episode.py:34)、[GUI 汇总](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/gui/legacy_backend.py:631)。

**P1/P2：缺失信息被伪装成正常观测，成功标签也是假定值。** 关节 exporter 把 -1/缺失夹爪读数钳成 0，混淆“没读到”和“全闭”；对每一帧写 is_success=true，并不检查任务结果。末端 converter 缺失夹爪指令会变闭合，缺失/损坏腕部图像补黑，外部图像读取失败则直接跳过该帧；它还按转换前帧数打印统计，可能与实际写入数不符。这会进一步压缩时间并引入没有 missing mask 的伪正常数据。应对必需模态失败关闭导出，或显式缺失掩码；success 使用人工/任务标签，未知不填 true。

**P2：记录点不足以宣称最终指令。** q_command 在 libfranka 二次处理之前。当前工作树明确打开 SDK 限速和默认低通，而且这是用户已有未提交修改之一；历史数据不能直接套用当前后处理假设。SDK 对 callback 输出的二次变换可见 [libfranka 0.21.3 control_loop.cpp](https://github.com/frankarobotics/libfranka/blob/0.21.3/src/control_loop.cpp)。建议分开保存 q_app_setpoint、控制器期望状态 q_d 等可得反馈，并说明各自对应的周期；q_d 也不能直接假定为“同一行刚发出的命令”。

夹爪同样存在 target_、commanded_、异步 move/grasp 与 width_ 四个层次，日志只保留 target_/width_。grasp 的布尔返回结果未记录，异常仅累计计数。缺少 request/send/complete 时间、指令 ID、结果和状态时间戳，因此目前只能评估期望值到缓存反馈，不能精确归因网络延迟、排队延迟和执行失败。[夹爪线程](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/src/pnp7_teleop.cpp:1006)。

**P2：同一时钟不等于同一物理时刻。** RGB 在取到 color frame 后打主机戳；机器人在回调时打主机戳。二者共用 CLOCK_MONOTONIC 解决时钟域问题，但没有消除曝光、USB 缓冲及回调延迟。1 kHz 日志最近邻通常天然能匹配到约半毫秒以内的主机样本；这不能证明图像里的物理姿态与该样本相差半毫秒。设备 timestamp 已保存，但未记录 timestamp domain、曝光信息或完成曝光到主机的标定。应保存有符号 skew、来源样本时间及 frame number/曝光 metadata，测量可观察的共同事件；人反应时延另行评估，不能直接套文档中的 100–300 ms。

事件流对“批到达时间/批末事件时间”作线性拟合，残差小不代表固定 USB 延迟小；拟合样本本身没有落盘，事件窗口也未检查是否超出实际事件覆盖范围。GUI 当前显式 --no-events 且没有启动事件 recorder；Shell 路径才尝试启动事件容器，并使用宿主另一个目录中的 record_events.py。实际事件采集软件版本需要单独追溯。

**P2：原始层有基础，但不完整、也不保证不可变。** 记录了 config.conf、calibration.json、相机序列号和完整时间日志，这是优点；但主臂原始 ticks/采样时间、clutch 原点、SDK 后处理指令、夹爪时刻及结果未完整保留。事件流在存前滤掉热像素；JPEG 是有损原始图像记录。这些都可以是合理取舍，但不应称“所有原始信号无损可重建”。

CSV 在内存积累到正常结束或可捕获的控制异常时才写盘，断电/SIGKILL/OOM 无法依赖该机制保住机器人轨迹。prune --yes 会删除未引用图片，且压缩 teleop.csv 后 builder 默认不读 .gz；GUI discard/过短轨迹会直接删目录。需要明确保留策略，并保证转换不修改唯一原始副本；不必为了格式名立刻迁移 MCAP，先做到增量可靠落盘、校验和、不可变归档及可重建。

**P2：元数据还不是完整数据契约。** 缺每条 episode 的 schema/软件 commit+dirty hash、转换版本、SDK/控制器固件、机器人型号与序列号、模型/工具坐标定义、相机内外参、采集时 task/operator/scene、人工成功标签、有效段与终止原因。config 的 sign/scale/filter 等已快照，但编译常量没有；复制 calibration.json 也没有核验其与 config 的生成关系。关节 exporter 默认 robot_type=panda，而 README 描述 FR3；这是明确的本体标签风险。任务文本在批量导出时统一赋值，需要确认同批任务确实一致。

**5. 不能机械套用教学文档的地方**

- “反馈当 action 必错”说得过满：应区分同帧状态回显与未来实际转移目标。PNP7 末端路径是后者，风险必须按执行接口验证，不能仅凭标签来源断言训练无效。
- “指令进 observation 必然泄漏”不是普遍定理，但可得性与是否容易形成退化闭环是两项独立检查。当前主要 live 路径可以取得动作历史，却仍可能把复制策略闭环，见上述新增 P1。应排查同帧新遥操作命令提前进入 state，并单独验收切换事件；是否删除或重定义该输入，要与训练、推理同时修改并对比。shadow 分支使用 width 推断，且 `else gripper_open` 会沿用旧值，宽度变小未必切换成闭合；它也没有将模型的夹爪输出递归输入下次观测。因此一次普通 shadow 跑无法直接证明 live 是否锁死。[推理 state](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/inference/pnp7_policy.py:694)。
- 当前 binary gripper 的“命令”应保持阶跃；连续物理宽度确实存在中间值，是否插值取决于反馈的时间可靠性。不能把所有 gripper_width 中间值当脏数据。
- 相邻关节 command 与 state 很接近，不足以判定记录错了；静止与正常跟踪都会这样。数据血缘加有激励的时延/误差测量才有解释力。

**6. 本次离线复现结果（全部为合成数据）**

| 复现 | 实际结果 | 能证明什么 |
|---|---|---|
| deadman=1、state=PAUSED | 行保留，build 返回 0 | 筛选没有约束 TELEOP |
| 两行间隔 1 秒 | 仍派生 dq_action；末端 converter 输出 0.05 m 单步增量 | 两条差分路径均未保护段边界 |
| 两行间隔 1 秒且位姿相同 | 仍输出六维全零的单步位姿动作 | 接缝无需表现为大位移，幅度阈值无法保证识别 |
| 585 行、两次夹爪切换 | 复制 state：584 标签中 99.6575% 正确，但切换帧 0% 正确 | 全局准确率掩盖切换失败；不是实际模型评测 |
| episode 最后一行 | dq_action=0.001 rad 的本行跟踪差 | 末行不是零，也不是合法“下一帧”目标 |
| 150 ms 间隔 | 关节 exporter 只识别 1 段 | 200 ms 阈值会遗漏短暂停 |
| 修改全部 q_command，并修改 gripper_width | 末端 state/action 均不变 | 此路径不使用关节目标和夹爪反馈 |
| -1 夹爪数据进关节 exporter | state/action 均变 0 | 缺失被误编码为闭合 |
| 单个关节目标 NaN | validator PASS / exit 0 | 有限数检查缺失 |
| 360 个非空坏图文件 | validator PASS / exit 0 | 图片存在检查不等于可解码检查 |
| wrist skew 500 ms | validator PASS / exit 0 | 校验没有执行相机同步门槛 |

复现脚本：[audit_data_semantics.py](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/diag/audit_data_semantics.py)。完整输出：[checks.json](/Users/andyliu/workspace/vla_data_collect/pnp7-lead-teleop/docs/data_semantics_audit_2026-09-09_checks.json)。这些结果证明代码存在可触发的漏洞，不代表真实历史数据已有这些缺陷或其发生比例。

**7. 质量判断与改进顺序**

| 维度 | 本次判断 |
|---|---|
| 实测反馈/应用目标记录 | 有良好基础，记录层级需要更精确命名 |
| 控制与图像时间戳 | 共用主机时钟是优点，物理时刻误差尚未验收 |
| 训练 action 语义 | 两条不同定义，必须分别立契约并绑定实际模型 |
| 分段、无效数据、终止 | 有明确可复现缺口，优先修复 |
| 自动质检 | 可做基础冒烟检查，不能作为高质量训练数据证书 |
| 溯源与重建 | 配置快照已起步，原始保留与版本元数据不足 |
| 真实示范成功率/覆盖度/视觉质量 | 没有实际数据，本次不能量化 |

建议顺序：

1. 确认实际训练数据与 checkpoint 的转换路径，为关节目标和末端实测转移分别建立契约。
   同时用真实切换片段测试夹爪复制基线、实际模型切换召回与时刻偏差；按实际 chunk 计算指标，再检查指令反馈闭环，不能将普通 shadow 无动作当作锁死证据。
2. 修复有效状态筛选、从原始状态机生成分段、禁止跨段差分和 chunk；保留源 timestamp/source row/有效性掩码。
3. 加入 finite、实际解码、时间单调/间隔、相机 skew 与缺失门槛；把 build/validate/export 的 verdict 串起来，明确 unknown success。
4. 补全夹爪新鲜度与发送结果、控制器期望状态、设备时间与时间映射证据；每条 episode 绑定采集与转换版本。
5. 用真实数据计算：分段内帧间隔 p50/p95/p99、相机重复/坏帧/缺失率、deadman=1 但 state!=TELEOP 的占比、关节跟踪 RMSE/p95/max 与滞后、夹爪命令到实际运动时延及抓取结果、跨段动作数、动作近零比例、任务成功率与场景覆盖。对 cmd-state 误差区分正常响应延迟和异常，夹爪 grasp 时不以到达零宽度作为成功条件。
6. 对末端反馈增量与明确目标指令两种标签做相同部署接口的短轨迹回放和受控对比；比较跟踪、停止/开合时机及任务成功，不能仅比较训练 loss。

本次只新增审计报告、合成复现脚本和结果，没有修改采集、导出、推理或机器人控制实现，也没有运行真机。

**8. 对独立评估的补充复核：哪些信息可以离线恢复**

独立反馈提出从 teleop.csv 重建滤波前目标，这是有价值的方向。对原始日志完整且具有前一行的 TELEOP 段，进入 TELEOP 时 q_origin=chain.target()，通常正是上一原始行 q_target。随后可以按以下定义重算：

```text
enabled[j] 为真：d[j] = clip(sign[j] * scale[j] * lead_delta[j], ±max_session_delta)
enabled[j] 为假：d[j] = 0
q_mapped_clamped[j] = clip(q_origin[j] + d[j],
                           kQMin[j] + kJointLimitMargin,
                           kQMax[j] - kJointLimitMargin)
```

这重建的是“死区之后、限幅之后、低通和运动学限速之前的映射目标”，不是未经处理的人手输入或心理意图。它不需要逆解低通，但必须满足以下条件：

- 使用该 episode 的真实 config（含 enabled_joints），以及采集版本中的编译关节限位和 margin；后者未包含在 config.conf 中。
- 原始日志未被截断。第一帧 FCI callback 只 seed、不写日志；若首条已记录行就进入 TELEOP，“段首上一行”并不存在，需另行验证初始化原点的恢复条件，不能无条件套用该公式。
- 用原始 state 转换识别段首，不能使用删行后的训练表猜测。`lead_delta` 已经过死区，且非 TELEOP 行写零，无法由其恢复全部主臂原始轨迹。

因此“很多现有日志可以离线修复”成立；“所有信息都在 L0、意图完全可逆”过强。原始主臂输入、最终 SDK 后处理命令、夹爪样本时间和实际发送结果、图像曝光到主机的时间关系仍有采集端缺口。

独立反馈中另外两处需要限定：

- `config.conf` 已随每条 episode 快照，因此 sign/scale/lowpass/vmax/amax 并非完全没有绑定。缺的是显式契约、软件/编译常量版本、L1/L2 的配置 hash/血缘与一致性校验。
- 用“位置差 ÷ 典型单步位移”估算的是等效步幅差，不能直接当作时间滞后，尤其在转向、限幅和速度/加速度饱和时。反馈给出的合成 p50/p95/max 尚无其脚本可复核，也不能迁移到真机统计。应按分段轨迹做时间平移拟合或事件响应测量，并同时报告无法用纯延迟解释的幅值残差。

后续 A/B 应在相同动作空间与部署执行接口比较 q_mapped_clamped 和 q_target；不能把 7 维关节目标直接替换到当前 7 维末端增量槽位，也不应绕过部署侧运动约束直接执行滤波前目标。
