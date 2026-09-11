# PI0.5：40 条完整 episode 重新训练

按照用户要求停止先前按暂停切成 44 段的训练。旧任务在第 151 步停止，
容器 `pnp7_pi05_40ep_20260910` 已退出。旧日志和数据保留用于追溯。

本轮每个原始 episode 对应一条训练轨迹，使用原转换器的
`--merge-deadman-gaps`：去除暂停时间，保持帧顺序，在 30 Hz 时间轴上串接。
跨暂停的动作依然由相邻保留帧的实测末端位姿计算。
40 条原始示教共 16,026 帧，生成 40 条 LeRobot episode、15,986 个 action 标签。
没有生成虚构示教或插入合成图像。

语言提示经用户确认：`pick up the blue cube and place it in the plate`。

- 原始数据快照：robot-s1 `/home/andyls/vla/pnp7/raw40_20260910`。
- 新训练数据：`/home/andyls/vla/pnp7/lerobot/pnp7_40merged_20260910`。
- 新模型与归一化：`/home/andyls/vla/models/pi05-pnp7-40merged-20260910`。
- 初始权重：与上一轮相同的 `RLinf-Pi05-LIBERO-SFT/model.safetensors`。
- 镜像：`rlinf-pnp7:ffmpeg`；训练容器：`pnp7_pi05_40merged_20260910`。
- 配置：`/home/andyls/vla/RLinf/examples/sft/config/pnp7_sft_pi05_40merged_20260910.yaml`。
- 运行记录：`/home/andyls/vla/pnp7/runs/20260910_40merged`。

训练于北京时间 2026-09-10 23:38 启动。日志目录：

```
/home/andyls/vla/RLinf/logs/20260910-15:38:34-pnp7_sft_pi05_40merged_20260910
```

启动验证已完成：到达第 5/7,500 个优化步骤，loss 和梯度范数均为有限值，
没有 traceback、CUDA OOM 或 NaN loss。交付时新容器正在运行，旧容器已退出。
这只确认训练已正常启动，训练尚未完成。

完成后的最终 checkpoint 预期位于：

```
<日志目录>/pnp7_sft_pi05_40merged_20260910/checkpoints/global_step_7500/actor/model_state_dict/full_weights.pt
```

参数沿用：LoRA rank 32、global batch 8、学习率 2.5e-5、warmup 250 步、
总计 7,500 步，每 2,500 步保存 checkpoint。由同一初始基模重新初始化，
不续接已停止的 151 步模型。归一化统计按重新合并后的数据重新计算。

输入仍为外部 RGB + 腕部 RGB + 10D state，输出为 10 步 × 7D action。
保留 robot-s0 字段适配：`rgb_cam2022` 作为外部相机，
Robotiq `gripper_requested_raw < 128` 映射为 1=open，其余合法值为 0=closed。

校验要求包括：40 个不同 source episode 一一映射、40 条 parquet、
15,986 个标签、4 个暂停缺口被串接、80 次夹爪开关切换、有限数值、
实际 OpenPI loader 的双相机输入、10 步 chunk 和 episode 末尾补齐。

查看进度：

```bash
ssh robot-s1 'docker logs --tail 30 -f pnp7_pi05_40merged_20260910'
```

检查容器：

```bash
ssh robot-s1 'docker inspect pnp7_pi05_40merged_20260910 --format "{{.State.Status}} exit={{.State.ExitCode}}"'
```
