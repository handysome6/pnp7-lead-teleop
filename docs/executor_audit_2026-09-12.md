# RLinf 实机执行审计与同步 libfranka 执行器设计

2026-09-12。结论：建议把完整 chunk 同步执行作为新的可解释基线：稳定保持 → 同时刻观测 → 推理期间保持 → 顺序执行 10 步 → 收敛后保持 → 下一次观测。每次采样新噪声是常见做法；当前更明确的问题是动作时间轴、累计目标、夹爪事件和实际跟踪之间的偏差。此方案不能保证修复模型自身的预测错误。

本次完成的是部署核查和重写契约，没有修改现有执行代码、启动 Home/FCI 控制或运行新的实机策略。通过 SSH 只读检查了 robot-s0、robot-s1。robot-s0 上的客户端与本地 `inference/robot_s0_policy.py` 内容一致；检查时没有发现正在运行的 teleop/policy/franka_control 进程。robot-s1 的固定 seed 与 seedexp 两个推理容器在运行。

远端源码和日志快照见 [sources.json](executor_audit_2026-09-12_sources.json)，含来源路径、SHA-256；定量核查见 [metrics.json](executor_audit_2026-09-12_metrics.json)。运行日志的 result 是退出原因，不是任务成功标签。

## 1. 实际部署链路

```text
双路最新 RGB + 实测 O_T_EE + Robotiq requested 开合状态
  → 224×224 RGB、10D state、任务文字
  → robot-s1 RLinf π0.5，返回 10×7
  → AsyncPolicy 不断取新观测并推理，单个请求在途
  → PrefixChunks 执行所选 chunk 的前 K 个动作，30 Hz
  → 在上次累计目标上加 delta
  → tracking governor 缩放/丢弃，或 tracking-stop=off 时 lead cap 裁剪
  → ROS equilibrium_pose
  → SERL 目标低通、误差裁剪、Cartesian impedance、力矩变化率限制
  → Franka

夹爪：同一 chunk 尾部 5 步均值 → 每 chunk 一票 → 连续两 chunk 同意才切换
```

- 模型输入不是 10 帧历史组成的 chunk：当前每次请求是一个 state、外部/腕部各一帧图像和 prompt；chunk 是输出的未来动作序列。
- `AsyncPolicy` 在机器人移动时取观测。推理结束立即开始下一次，并不会等待执行器完成。
- `PrefixChunks` 完成已选前缀后挑最新 chunk；其余预测可能完全未被选择。K=10 仍不是同步循环。
- 新 chunk 的 observation pose 虽保存在 packet 中，选择动作时没有作为累计目标的新原点。`p, rotation` 跨 chunk 持续累加。
- governor 缩小动作后不补回；关闭 governor 后 `cap_lead` 仍把目标限制在实测位姿前方 20 mm / 0.10 rad，多余部分丢弃。
- 等待新动作时不继续发布，250 ms 后 SERL watchdog 清除位姿误差。这不是“确认终点到达后保持终点”。
- 过期判断是 observation age − action_index/30 > 400 ms；不等价于延迟补偿。同步新实现应依据自身观测/执行时序重新定义 freshness，不能直接复用此规则。

代码依据：`inference/robot_s0_policy.py` 的 `AsyncPolicy`、`PrefixChunks`、`run`；`inference/tracking_governor.py`；`inference/robotiq_policy.py`；远端快照 `serl_controller.cpp`。

### 一次已经做过的 K=10 运行

`live_full_20260912_014036_2038713.json`，29.738 s，以松脚踏结束：

| 项目 | 日志计算结果 |
|---|---:|
| 请求/记录的推理次数（含预检查等） | 173 |
| 被执行器选中的 chunk | 89 |
| 发布动作数 | 886 |
| 推理往返中位数 | 174.95 ms |
| 发布动作时 observation age 中位数 | 411.06 ms |
| 平移被 lead cap 裁剪的控制步 | 206 |
| 每步裁剪距离范数之和 | 293.00 mm |

293 mm 是各步被裁掉距离的累计标量，不是单次误差、终点误差或机器人少走的净位移。最后一个 chunk 也可能随脚踏释放中断。该日志足以证明“每次选 10 步”仍不能保证原始轨迹被执行。

SERL 的 `filter_params_=0.005` 同时用于位置和姿态目标滤波；若 update 为 1 kHz，一阶时间常数约 199.5 ms。配置平移刚度 2000 N/m，误差逐轴裁到 10 mm，因此弹簧项逐轴上限约 20 N；这不是总末端作用力的硬上限，阻尼等项另计。该滤波、误差裁剪和控制律才是影响响应的具体层，不能把所有误差笼统归因于 ROS 通信。

## 2. Seed：新噪声常见，每请求重置相同 seed 才是特殊实验

robot-s1 的 `serve_pi05_seed.py` 读取请求 seed，在锁内设置 torch/CUDA RNG 后推理。客户端每次运行生成 base，随后按 base+k 递增。上面的 173 个样本有 173 个不同 seed。固定服务则每次都重置 seed=0；本地默认参数仍是 fixed，实际这次日志选择的是 per-request。

RLinf `sample_actions()` 在没有显式 noise 时生成初始噪声；eval 选择 flow ODE，仍不意味着初始噪声被固定。不要混淆动作 horizon=10 和 denoising steps。

[OpenPI 官方 Policy](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/policies/policy.py) 的 JAX 路径每次推理 split RNG，支持显式传入 noise。由此支持的结论是“每次推理用新随机样本是正常实现”，不是所有机器人都必须采用某个 seed 协议。

建议：

- 正常评测：固定 run_seed，得到可复现的 chunk_seed 序列；记录 chunk ID、seed、原始输入/输出和模型版本。固定 seed 本身不保证跨 GPU/软件版本逐位复现。
- 调试：保留每请求固定同一个 seed/noise 的模式，用于同观测复算和控制器对比。
- 不根据一次失败动态换 seed 重试，不把多个 seed 的动作平均当作可执行轨迹；不同采样可能代表不兼容的抓取/释放策略。
- 变 seed 并不保证 chunk 连贯；固定 seed 也不保证跨观测连贯。先把执行语义固定，随后在相同初始场景、多条预设 seed 序列上比较。

## 3. 执行多少步与循环频率

区分三个频率：训练/动作时间轴 30 Hz、模型闭环重规划频率、FCI 回调约 1 kHz。10 步表示名义 333 ms 的运动；不应因为推理要 175 ms，就把每个 delta 当作 175 ms 的动作，也不应把 delta 误当速度。

设 K 为执行步数，Δt=1/30 s，L 为观测传输/推理往返耗时，S 为终点收敛及取到新图像等额外等待：

```text
T_cycle = K Δt + L + S
f_replan = 1 / T_cycle
每秒消费动作数 = K / T_cycle
```

仅代入 L=175 ms、S=0 作理想预算（并非新的实机测量）：

| K | 名义运动时间 | 同步重规划频率 | 平均消费动作数 |
|---|---:|---:|---:|
| 2 | 66.7 ms | 4.14 Hz | 8.28/s |
| 4 | 133.3 ms | 3.24 Hz | 12.97/s |
| 6 | 200.0 ms | 2.67 Hz | 16.00/s |
| 10 | 333.3 ms | 1.97 Hz | 19.67/s |

K 大：更多预测被执行、推理暂停占比小，但开环区间更长、远期误差更可能影响任务。K 小：更快重看现场，但高推理延迟下有效运动变慢，也可能总在动作前缀徘徊，尚未执行到夹爪切换阶段就重新预测。

[Diffusion Policy](https://diffusion-policy.cs.columbia.edu/) 使用 receding horizon；[OpenPI ActionChunkBroker](https://github.com/Physical-Intelligence/openpi/blob/main/packages/openpi-client/src/openpi_client/action_chunk_broker.py) 在配置的执行 horizon 耗尽后才重新调用 policy。不存在通用“执行两步”或“必须全执行”的数值标准。

[Physical Intelligence 的 RTC 说明](https://www.pi.website/research/real_time_chunking) 明确说 π0/π0-FAST/π0.5 曾同步执行：一个 chunk 完成，等待推理，下一个 chunk 从静止开始；同时指出这种暂停不在训练数据中，影响效率和动态任务。RTC 是进一步处理异步延迟与 chunk 一致性的方案，不能用当前后台推理线程直接等同于 RTC。

对当前重写：默认 K=10、30 Hz 连续轨迹、边界保持。先测清执行误差，再把 K=4/6/10 放在同一同步执行器上做对照；不要把旧异步 K=10 的表现当成新方案的结论。选择 K 时看延迟 P50/P95、轨迹误差随预测时域增长、夹爪误触发、完成时间及任务成功率，不凭循环频率一个指标决定。

## 4. 与采集一致：控制模式和数据语义比库名更重要

采集端现用 `franka::JointPositions` + `ControllerMode::kJointImpedance`，同时有 PNP7 输入映射/滤波/运动约束，以及 libfranka 的 rate limit 和低通。当前模型却是 EE 动作，不是该采集控制器的 q_target。

已经读取 robot-s1 本轮真正的转换器，标签为：

```text
state_t = [p_actual_t, R_actual_t 的前两列, g_requested_t]
action_t = [p_actual_(t+1) - p_actual_t,
            RPY(R_actual_t.T @ R_actual_(t+1)),
            g_requested_(t+1)]
```

即前六维是相邻实测 EE 转移，夹爪是绝对二值指令（1=open）。RPY 对应矩阵组合 `Rz(yaw) Ry(pitch) Rx(roll)`；不要因 converter 的文字注释而混用 intrinsic/extrinsic API。平移在基座系相加；旋转右乘局部相对旋转。

从本 chunk 观测位姿一次性还原全部 waypoint：

```text
p[0], R[0] = observation 的实测位姿
p[i+1] = p[i] + delta_xyz[i]
R[i+1] = R[i] @ Rz(dyaw[i]) @ Ry(dpitch[i]) @ Rx(droll[i])
```

不要全部相对 p[0] 单独相加；不要跨 chunk 接上旧累计目标；不要每执行一步又用实测位姿重新积分，避免把跟踪误差偷偷变成目标重定义。

### 推荐第一版接口

采用 C++ 长驻 libfranka 0.21.3 控制进程，优先验证原生 `CartesianPose` motion generator + `kJointImpedance`。该组合由 [libfranka 0.21.3 Robot API](https://github.com/frankarobotics/libfranka/blob/0.21.3/include/franka/robot.h) 明确支持，使用机器人内部的笛卡尔到关节运动生成。这样维持采集所用的内部关节阻抗模式，并去掉 SERL 自定义力矩控制律。

这仍不等于完全复刻采集的 `JointPositions` 输入：还要验证内部逆解、肘部/冗余姿态、奇异点、关节约束和实际 EE 跟踪。如果一定要求相同 JointPositions 输入，可显式用连续 IK 生成 q 轨迹，但那会新增必须验证的映射；现有 6D EE 模型本来也不能唯一恢复示教的 7 个关节姿态。第一版不建议无理由自写一套 IK。

保留相同工具/负载、基座/EE 定义、内部控制设置与 FCI 约束，并记录 `F_T_EE`/`NE_T_EE`、`EE_T_K`、负载/质心/惯量、软件版本。旧数据的配置要单独核实；当前采集代码在训练数据之后新增了陷波，不能将当前所有滤波机械套到旧数据。EE 标签已包含当时实际响应，重新施加主臂专用死区、映射或滤波会再次改变轨迹。

## 5. 同步执行器的完整契约

```text
HOLD/SETTLE → CAPTURE → INFER（保持） → VALIDATE → EXECUTE[0..9]
     ↑                                                 ↓
     └──────────── 终点收敛并确认夹爪状态 ────────────────┘
```

1. **长驻实时环**：约 1 kHz 回调在 HOLD 和 INFER 阶段继续运行。网络、图像、磁盘和串口在非实时线程；模型只允许一个在途请求。用有界缓冲交接整块轨迹，按 ID 确认接收/开始/完成；不重复执行、不在运动中替换 chunk。
2. **稳定后采样**：上一个 chunk 的轨迹时钟走完不算实际完成。终点位置/姿态误差与实测速度连续满足阈值，才发布完成。阈值、持续时间、超时需用回放测量确定。夹爪闭合遇物体可算正常完成，不要求实测宽度为零。
3. **观测一致性**：等到稳定时刻之后的双路新帧，与机器人缓存状态按时间对齐；记录曝光/接收时间与 skew。实时状态只能从当前 FCI 回调缓存取得，不在另一线程对同一 Robot 调 readOnce。等待结果时保持，返回后若环境/位姿已明显改变，拒绝这块并重新观测。
4. **chunk 内连续运动**：按 30 Hz 的 knot 时间生成连续的位置/旋转轨迹，再以 1 kHz 取样；只在 chunk 边界收敛停止，不在十个 waypoint 之间逐个阻塞“到点停住”。旋转在 SO(3) 上插值，不能直接插值绝对欧拉角。
5. **端点与时间约束**：静止起止和原始 30 Hz 路径可能无法同时满足速度/加速度/jerk 约束。轨迹生成必须显式处理；优先保持全部 waypoint 和顺序，必要时采用有日志的时间拉伸，或报告不可执行并终止，不能静默缩小 delta/裁剪目标。时间拉伸改变训练时序，应计入评测。
6. **真实 hold**：平滑收敛后保持终点参考。把每个 callback 的 measured pose 重新当 hold target 会导致顺从漂移；反之也不能在仍运动时直接冻结并要求瞬间停住。下一块参考以新观测锚定，起始命令仍须与 FCI 已接受的命令连续；若二者有残差，须处理并记录，不能突然跳目标。
7. **夹爪逐步解码**：第一版显式定义 `g>=0.5 → open`，其余 close，匹配二值标签；没有跨 chunk 均值或一致性投票。根据 `g[i+1]` 的下一状态语义确定并记录事件时间。连续相同值可合并为保持；不同值的切换不能被 latest-only mailbox 覆盖。分别记录目标、实际发送、设备确认、完成/接触时间。
8. **夹爪物理边界**：每步输出均被解释，不代表夹爪能在 33 ms 内完整开合。若输出开/关抖动超过设备可执行速度，应记录事件追踪失败，而不是隐藏在投票里。hold/推理期间仍维持最后合法夹爪命令和通信，不应把正常等待误判成当前驱动的 250 ms 失联。
9. **保持模型输入语义**：手臂使用实测反馈；当前 checkpoint 的 state 最后一维仍取合法 requested 开合状态，不突然替换成实测宽度。宽度、接触、fault 是独立诊断量；输入语义修改应伴随训练/评估更新。
10. **显式失败**：不可达/越界/超时/跟踪失败/脚踏释放/机器人异常都中断并报告原因，保留已执行索引；不因“全部执行”而掩盖故障，也不恢复后自动重放。模型超时与实时控制失联分开判断。

因此“忠实执行”应验收为：动作含义不变、完整路径与事件可追踪、执行修改可量化、实际误差在声明范围内。不是声称物理系统可以零误差、零插值或不受运动限制。

## 6. 验证顺序与成功率边界

先用真实数据验证 converter 的 delta 累积可还原未来 O_T_EE，再用同一小段已知轨迹分别测新后端的原始 waypoint、插值参考、SDK/机器人期望状态和实测状态，记录时延、均值/P95/最大跟踪误差和终点收敛时间。之后再运行模型闭环；这样能区分预测不对和执行没跟上。

关键检查包括：右乘旋转、10 步一次且仅一次、hold 后取新图、请求 ID/重复响应、首末命令连续、夹爪事件不丢失、超时/中止不续跑。不能只断言发送了十条消息。

同一模型、任务摆放和预设 seed 序列下对比：当前实现、同步 K=10、同步 K=4/6；分别报告成功率、完成时间、夹爪事件错误和跟踪误差。多次重复，不用单次成败选参数。

已有离线诊断（`artifacts/offline_eval_20260911/`，随分析产物留在本机，未纳入仓库）在固定 seed 服务、三个训练 episode 的 237 个窗口上，10 步位置误差均值 14.69 mm、P95 37.76 mm。这是预测与示教的误差，不是控制器跟踪误差，也不是新 per-request/sync 配置的评测。它说明单独更换执行器不足以承诺成功率；严格执行也可能更完整地执行错误的抓取或开爪。

当前动作采用实测 EE 差分，所以示教实际出现的振动也可能进入动作标签；不要套用关节 q_command 导出路径“动作里没有从臂振动”的结论。训练的 40merged 版本还串接了四个暂停缺口，这些都应作为模型侧解释保留，而不在执行器内用启发式补偿掩盖。
