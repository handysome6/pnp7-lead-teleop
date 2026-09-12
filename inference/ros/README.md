# Isolated ROS deployment on robot-s0

## Current manual validation entry point (2026-09-11)

With the rebuilt ROS launch running, on **robot-s0**:

```bash
cd ~/workspace/pnp7-lead-teleop/inference/ros
~/.pixi/bin/pixi run validate
```

If ROS is not running, first run `~/.pixi/bin/pixi run control` in a separate
terminal in this directory. Stop the old launch with Ctrl-C before replacing it;
never start two control launches. Controller changes require a restart.

`validate` selects live/full, native learned deltas, 40 seconds by default (up to 90 with `--duration`)
(not maximum hardware joint speed). Each live invocation, including direct Python
and `pixi run live`, first disconnects the idle ROS hardware, runs existing
`bin/pnp7_teleop home conf/full100b.conf`, reconnects and verifies all seven joints
within 0.01 rad, then prepares observations/inference. The configuration is read
from robot-s0 each time; no Mac Home coordinates are copied. Failure forbids
inference; failed/interrupted Home leaves ROS disconnected. Restart the ROS launch
after investigating such a failure. A process lock prevents concurrent clients.

**The command starts Home motion before pedal arming.** Clear the workspace and
keep the physical stop within reach. Release the pedal for Home; the pedal does
not stop Home. Ctrl-C requests its existing smooth stop. After `HOME_VERIFIED`
and `LIVE_READY`, hold the pedal to run the policy; release ends this run.
No automatic retry or robot error recovery. Gripper state is not reset by Home.

Tracking guards are now exactly three times their previous values: client
15 mm / 0.10 rad -> **45 mm / 0.30 rad**; controller 30 mm / 0.20 rad ->
**90 mm / 0.60 rad**. The 250 ms controller watchdog, timestamp checks,
workspace limits, controller filter and hardware collision reflexes are
unchanged; translation stiffness and error clip were raised later (see Controller stiffness).
The larger lag allowance is not proof of successful tracking.
Logs: `validation/live_full_<timestamp>_<pid>.json`, including verified Home and
tracking limits. `pixi run shadow` remains a zero-motion observation diagnostic;
it does **not** Home and is not the live-validation entry point.

The sections below describe earlier tests and historical settings.

### Gripper chunk voting (2026-09-12)

Each policy chunk casts one gripper vote, open at >= 0.75 and close at <= 0.25.
The Robotiq target switches only after two consecutive chunks agree; further
actions from the same chunk only keep the GoTo command fresh. The vote value is
the mean of the chunk's last 5 predicted gripper steps (`--gripper-vote tail`, the
full-profile default: the state the chunk plans to end in) or of its whole 10-step
horizon (`--gripper-vote chunk`). `--gripper-vote action` restores the old
per-action two-in-a-row debounce. Switches are logged as `gripper_switches`, and
each target row records `gripper_command`.

Tail voting replaced the whole-horizon mean after
`live_full_20260912_010356_2029813`. That run had no drop, but the arm hovered 55 s
over the plate (median +45 mm above the demonstrated release height, predicted dz
about 0) and never released: only 2% of its chunks had a whole-horizon mean
>= 0.75. Across six runs the tail vote cast one open vote in 490 transport chunks,
never twice in a row; in that run it first agreed twice at 57.7 s, when the arm
dipped to +20..+40 mm. Earlier "successful" releases came from the same sporadic
open-first samples that dropped cubes in transport, so they were partly luck.
Release remains a policy weakness.

Reason: with per-request noise seeds a single chunk occasionally starts with
several "open" steps and then returns to closed (e.g. `[0.98, 0.94, 0.89, 0.98,
0.96, 0.10, 0.04, ...]`). Executing 2-4 of its steps satisfied the old debounce on
its own. That dropped the cube mid-transport in
`live_full_20260912_003634_2024928`, `live_full_20260912_004721_2026800` (128 mm
from the plate) and `live_full_20260912_004941_2027155` (81 mm) while Robotiq still
read 69-75 (holding). Replaying chunk voting on the same predictions rejected each
trigger (chunk means 0.30, 0.50 and 0.66/0.53), removed the open/close chatter at
grasp, and still released over the plate in the two successful runs, about 0.7 s
later. The replay cannot model the closed loop after the first divergence.

### Longer runs and action prefix (2026-09-12)

The full profile now allows `--duration` up to **90 s** (`validate` still defaults
to 40 s) and `--actions-per-inference 2..10` (default 2; 10 executes whole chunks).
Action k of a chunk is intended for k/30 s after its observation and runs at most
400 ms later than that; otherwise the rest of that prefix is skipped. (Until
2026-09-12 the bound was on observation age alone, which capped useful prefixes at
about 4 actions.)

```bash
~/.pixi/bin/pixi run validate --client-tracking-stop off --noise-seed per-request --duration 90
~/.pixi/bin/pixi run validate --client-tracking-stop off --noise-seed per-request --duration 90 --actions-per-inference 4
```

Reason: `live_full_20260912_002230_2022414` (slack cables, 40 s) reached 6 mm in xy
and +7 mm above the demonstrated grasp height, but ran out of time while closing.
Gripper outputs near the grasp were mostly 0.99-1.00 with isolated close samples,
so the two-update debounce first switched at 39.62 s and the run ended at 40.02 s.
The final descent was slow: the policy asked for 0.3-1.9 mm per 10 steps, and only
2 of 10 steps ran per ~160 ms inference.

### Controller stiffness (2026-09-12)

`robot_s0_impedance.yaml` translation is now **2000 N/m with damping 89** (was
1000 / 63; SERL's default pair, same damping ratio). The translational error clip
is now **10 mm** (was 5 mm), so the pull is at most about 20 N per axis; rotational,
nullspace and watchdog settings are unchanged. Restart `pixi run control` to load
changes.

Clip reason: in `live_full_20260912_011939_2032689` and
`live_full_20260912_012131_2033027` the arm sat 48 s and 77 s at the grasp posture
without closing. The policy kept commanding a little +x (+0.5..+1.4 mm per 10
steps), the leashed target led about 17 mm in +x (past the 5 mm clip, ~10 N) and
the arm crept 0.2-0.4 mm/s, even when 3 mm from the demonstrated grasp point. The
y lead was only 2-3 mm and the arm moved freely earlier in the approach, so the
resistance appears at the extended pose (gravity model or cables), and the policy
kept waiting to align.

Reason: in `live_full_20260911_235005_2016668` (per-request seed, 40 s) the arm
stopped moving horizontally about 36 mm above grasp height, touching nothing.
From 8 s the leashed target led by +8..+18 mm in x and in y (past the clip, about
5 N per axis) toward the grasp point. Publication gaps stayed at or below 200 ms
and joints were at least 1.07 rad from their limits, yet horizontal speed was about
zero; similar leads had moved the arm at 9-16 mm/s while it was already moving.
Static friction beat the pull, so the last 1-2 cm of lateral correction never
executed, and the policy, still waiting to align, neither descended nor closed.

### Checkpoint selection (2026-09-11)

Each 40merged checkpoint runs in its own robot-s1 container. The client picks the
port with `--policy-step` and refuses a server whose health reports a different
model id. 7500 (default) is `pnp7-pi05-40merged-inference` on 5559; 2500 is
`pnp7-pi05-40merged-step2500-inference` on 5560; 5561 is reserved for 5000. All
use the same base model and normalization.

```bash
~/.pixi/bin/pixi run validate --client-tracking-stop off --duration 20 --policy-step 2500
```

The step-2500 container reuses the 7500 container's image, read-only mounts and
arguments; only port, model id and weights differ:

```bash
run=/workspace/RLinf/logs/20260910-15:38:34-pnp7_sft_pi05_40merged_20260910
docker run -d --name pnp7-pi05-40merged-step2500-inference --gpus all --network host \
  --shm-size 16g -w /workspace -e PYTHONPATH=/opt/pnp7:/workspace/RLinf \
  -v /home/andyls/vla/models:/models:ro \
  -v /home/andyls/vla/pnp7/inference40_20260911:/opt/pnp7:ro \
  -v /home/andyls/vla/RLinf:/workspace/RLinf:ro \
  rlinf-pnp7:ffmpeg /opt/venv/openpi/bin/python /opt/pnp7/serve_pi05.py \
  --bind 100.71.83.59 --port 5560 --model-id pi05_pnp7_40merged_step2500 \
  --base-model /models/pi05-pnp7-40merged-20260910 --config "$run/tensorboard/config.yaml" \
  --weights "$run/pnp7_sft_pi05_40merged_20260910/checkpoints/global_step_2500/actor/model_state_dict/full_weights.pt"
```

The step-2500 container was removed after its live test (107 mm short in y, run
with seed 0); recreate it with the command above if needed.

### Per-request noise seed (2026-09-11)

`serve_pi05.py` resets torch seed 0 before every inference, so the deployed
policy always starts flow matching from the same noise. An offline experiment
(robot-s1 `/home/andyls/vla/pnp7/seed_exp_20260911`: 111 approach-phase training
windows from 11 episodes, 8 seeds) found that this noise shifts every 10-step
prediction by dy -3.2 +/- 0.1 mm and dz -7.5 +/- 0.2 mm against the mean of seeds
1-7, in all windows and episodes. Seed 0 predicts -69% of the demonstrated lateral
motion; the 8-seed mean predicts 95% with about zero bias. This matches the
repeatable live -y shortfall and fast descent.

```bash
~/.pixi/bin/pixi run validate --client-tracking-stop off --duration 20 --noise-seed per-request
```

`--noise-seed per-request` (7500 only) uses `pnp7-pi05-40merged-seedexp` on port
5562. It is started like the step-2500 container above, but mounts
`/home/andyls/vla/pnp7/seed_exp_20260911` as `/opt/pnp7` and runs
`serve_pi05_seed.py` (a copy of `serve_pi05.py` that accepts a `seed` per
request) with `--port 5562 --model-id pi05_pnp7_40merged_step7500_seedexp` and
the global_step_7500 weights. Its seed-0 output is identical to port 5559. The
client refuses a server whose health lacks `per_request_seed`, because deployed
servers silently ignore the field. Each run draws a new base seed; inference k
uses base + k, logged as `noise_seed_base` and per-sample `noise_seed`.

Offline evaluation shares these servers; never run it during a live robot run.

### Observation freshness (current, both modes)

No action runs more than 400 ms after its intended time (observation time plus
index/30 s; until 2026-09-12 the bound applied to observation age alone). A chunk
whose next action would be later than that is dropped for the rest of its prefix
and logged (`STALE_ACTION_SKIPPED`, `skipped_stale_actions`) while the client
waits for a strictly newer prediction. The run ends only when the newest observation
is older than 1.0 s (stuck worker, server or network); a single round trip above
800 ms still fails immediately. Nothing is published while waiting, so after
250 ms the controller watchdog drops the target and the arm stops.

Previously any newest observation older than 400 ms ended the run, even on a
waiting tick where nothing was executed. Just before a new chunk arrives that age
equals two consecutive inference cycles, leaving about 20 ms of margin at a
190 ms round trip: `live_full_20260911_222709_2001670` stopped at 411 ms while
the arriving chunk was about 240 ms old.

### Diagnostic run without client tracking stops (opt-in)

```bash
~/.pixi/bin/pixi run validate --client-tracking-stop off --duration 10
```

Full profile only. No two-stage governor, no 45 mm / 0.30 rad client stop and
no persistence stop. Policy deltas are applied unscaled, but each published
target is leashed to the measured pose: at most `--target-lead-cap-mm`
(default 20) and `--target-lead-cap-rad` (default 0.10) ahead, keeping the lead
direction and rotation axis. Excess motion is dropped, never buffered, and
logged per tick in `tracking_samples` (`dropped_translation_m`,
`dropped_rotation_rad`).

Why the leash: in `live_full_20260911_221123_1998711` (no leash) the arm,
limited to about 25–37 mm/s by the 5 mm clip, lagged up to 3 s behind the
integrated target. The policy, observing that lagging arm, kept commanding the
first descent; at 90 mm lead SERL froze the arm, and the target alone ran about
2.5 cm below the demonstrated grasp height until the workspace check stopped the
run. With the leash the policy observes an arm that is near its target. Lead
beyond the 5 mm clip plus filter lag adds no pull, so arm speed should be
essentially unchanged; the next run measures this.

Tracking error is still computed every tick and printed on crossings as
`TRACKING_UNGUARDED level=1` (beyond 45 mm / 0.30 rad) or `level=2
controller_ignores_target=true` (beyond the controller's 90 mm / 0.60 rad: SERL
then holds the measured pose while the gripper still receives policy commands;
release the pedal). With the leash these mean the arm is being pushed or
blocked, not ordinary lag.

Unchanged: SERL error clip (10 mm, at most about 20 N per axis at 2000 N/m), 250 ms controller
watchdog, demonstrated-workspace check on the target, action outlier bound,
full-run rotation limit, stale-data checks, deadman pedal and collision
reflexes. The operator's physical stop is the tracking safeguard in this mode;
do not raise the clip or stiffness further without a new review. The default remains
`--client-tracking-stop on`.

### Two-stage tracking governor (current default)

Client hard stops remain **45 mm / 0.30 rad**; the controller-side guard remains
90 mm / 0.60 rad. Before reaching the client hard boundary, a command governor
now reduces target increments continuously through a smoothstep curve:

- Full increment below 15 mm and 0.10 rad tracking error.
- Progressively reduced increments between 15–30 mm or 0.10–0.20 rad.
- No new arm targets or gripper actions at 30 mm or 0.20 rad. Existing command
  watchdogs are not refreshed while paused; gripper GoTo is cleared by its
  existing stop path. This is not a promise of an instantaneous physical stop.
- A separate per-action budget prevents even a large candidate delta from
  creating more than 30 mm / 0.20 rad target lead (if already beyond this soft
  envelope, no new action is applied).
- Deviation at/above 15 mm or 0.10 rad starts a **1-second** timer, including
  waiting ticks. It resets only once BOTH errors fall below 12 mm / 0.08 rad.
  Persistent deviation or the unchanged hard threshold stops the run.
- Recovery of the increment multiplier is limited to 2/s (zero to one takes
  at least 0.5 s). Reductions can act immediately to preserve the lead bound.

Scaled-off and paused deltas are discarded, never buffered or caught up later.
The stored target advances only by the applied delta; it is not snapped to the
measured pose. These are conservative diagnostic parameters, not certified safety
limits or proof of successful closed-loop operation. Workspace, stale-data,
deadman and collision safeguards are unchanged. Per-tick `governor_samples`,
`governor_last` and threshold settings are saved in the run JSON. Target rows
distinguish original `policy_action` from scaled `action` and record its scale.

Start with the manually supervised five-second run:

```bash
~/.pixi/bin/pixi run validate --duration 5
```

### Two-step inference prefixes (current validation behavior)

The full client still requests predictions asynchronously, but executes only
indices **0 and 1** of each selected chunk, separated by a 30 Hz control tick.
It then waits for a strictly newer prediction and discards indices 2..9. A new
prediction does not interrupt the selected two-step prefix; after that prefix,
the newest available chunk is selected without queuing old predictions.

Waiting does not accumulate deltas, repeat actions, publish pose targets, or
refresh gripper commands. Camera/state/pose tracking checks continue; existing
250 ms command watchdogs remain effective; observation age is handled as in
"Observation freshness" above.
The target is still continuous across chunks: this reduces accumulation rate,
not the accumulated error itself. With observed 160–200 ms inference round trips,
expect roughly 10–12.5 new actions/s rather than 30, with nonuniform spacing.
The 40-second duration is unchanged, so task progress may be slower.

`LIVE_READY` and the JSON log report `actions_per_inference=2`; target log indices
must only be 0 or 1, and `waiting_ticks` counts idle ticks after a prefix. The
smoke profile also consumes only two actions per prediction, retaining its
existing amplitude limits. No tracking or collision safeguards were removed.

### Serial preflight startup ordering

After two post-Home preflights stopped with `Write timeout` before pedal arming,
read-only measurements found ~159 ms Python-thread scheduling gaps during camera
initialization, exceeding the 80 ms serial deadline. This is a suspected startup
contention source, not proof that every timeout has the same cause. The client
now waits for both cameras to finish startup/warmup before opening the gripper
status worker. It prints `CAMERAS_READY` before this step. The 80 ms serial
deadline and all live safeguards remain unchanged. A stopped serial worker now
fails preflight immediately, with function/phase/elapsed-time diagnostics; it is
not automatically restarted. `diag_startup_readonly.py [--staged]` measures the
two startup orders without Home, controller activation or FC16 output writes.

Reuse the ROS Noetic `franka_control` + SERL Cartesian impedance controller;
do not replace the working teleoperation environment or introduce a new FCI bridge.

Source snapshots copied from pnp7 on 2026-09-11:

- `/home/franka/catkin_franka/src/franka_ros`, clean tag `0.10.0`.
- `/home/franka/catkin_franka/src/serl_franka_controllers`, commit
  `1f140ef0d8e3fc443569c193d3ede1856e50d521` (untracked notes/tests retained).
- `/home/franka/catkin_franka/libfranka-0.21.3`, including its `common` submodule.
  Main commit `85912fe02258d8cb811d3eff1f11e52ce89e3217`, common commit
  `2e090a65e51c94a98f9fc82b6ec3d1aa54d0f85f`.

The previous ROS binary links libfranka 0.15 and cannot be reused following the
robot firmware update. Prebuilt conda libfranka 0.21.3 requires Poco 1.15.3,
whereas RoboStack Noetic class_loader requires Poco 1.15.0. Build the same
libfranka source against the isolated ROS environment instead of mixing ABIs.
The resolved environment uses Python 3.12, Eigen 3.4, and Pinocchio 3.9.0.
`compat.patch` records the only source adaptations: C++17 for franka_hw/control,
an explicit cstdint include, and honoring CATKIN_ENABLE_TESTING in description.
The initial build did not change SERL's controller source. The live-validation
additions below now add target guards while retaining the impedance control law.
The manifest pins Empy
3.3.4 because ROS Noetic's message generator does not support Empy 4.

On a fresh source copy, apply once with `patch -d src -p1 < compat.patch`.
The robot-s0 source snapshot already has these changes applied.

Build on robot-s0 (does not connect to hardware):

```bash
cd ~/workspace/pnp7-lead-teleop/inference/ros
~/.pixi/bin/pixi run build
~/.pixi/bin/pixi run bash -c 'source devel/setup.bash && python check_build.py'
```

## Verified live smoke test, 2026-09-11

Both reconnected cameras were identified: external `317622072022`, wrist
`233622071437`. Camera warmup is required: initial auto-exposure/white-balance
frames were visibly dark/green; the client now waits before taking observations.

- Two shadow runs passed with zero pose targets and zero gripper output-register writes.
- A supervised, pedal-gated 5-second run completed: **51 pose targets**, 19
  inference samples including preflight, approximately **2.50 mm net measured EE
  movement**. Mean model inference 139 ms; mean network round trip 186 ms.
- Final robot mode idle; impedance controller stopped; no current or last-motion
  errors; no cleanup errors; Robotiq fault0.
- The gripper stayed open throughout. Physical grasp/release, physical early
  pedal-release stopping, and full pick-and-place success are **not yet validated**.
- C++ watchdog tests and four Python tests passed, covering coordinate semantics,
  action bounds, gripper debounce/dead-man/staleness, CRC, and ROS serialization.

Run log on robot-s0:
`validation/live_5s_40merged_20260911_c.json`.
Earlier `live_5s_40merged_20260911.json` and `_b.json` were cancelled during pedal
waiting with zero targets sent, not motion trials.

## Start / repeat on robot-s0

Never run collection and ROS hardware control simultaneously. Leave the pedal
released while starting. The following bringup starts **only the state controller**
and loads the impedance controller inactive. No Franka-Hand driver is started.

Terminal 1, only if this ROS launch is not already running:

```bash
cd ~/workspace/pnp7-lead-teleop/inference/ros
~/.pixi/bin/pixi run bash -c 'source devel/setup.bash && exec roslaunch robot_s0.launch'
```

Terminal 2, shadow without arm/gripper motion:

```bash
cd ~/workspace/pnp7-lead-teleop/inference/ros
~/.pixi/bin/pixi run shadow
```

Supervised 5-second motion smoke test:

```bash
~/.pixi/bin/pixi run live
```

Wait for `LIVE_READY`, then hold the pedal. First release or timeout ends the run.
Keep the physical stop accessible. This conservative profile is not a full-task
evaluation: limits are 1 mm/0.003 rad per target, 5 cm/0.15 rad from start, and
3 actions per observation at 30 Hz, with pauses for synchronous inference.
Workspace and action outlier bounds come from the exact 40-episode norm stats.
The server must identify itself as `pi05_pnp7_40merged_step7500`.

Stop the ROS launch with Ctrl-C before reopening collection. The policy server on
robot-s1 is Docker container `pnp7-pi05-40merged-inference`, listening on the
Tailscale address `100.71.83.59:5559`. No checkpoint was changed.

## Runtime changes and reproducibility

`runtime_guard.patch` adds an explicit option to preserve the robot's existing
collision settings, a nonblocking RT target buffer, a 250 ms controller-side
watchdog, timestamp/frame/quaternion validation, and 3 cm/0.2 rad target-error
guards. Integral state is initialized and cleared for this zero-integral profile.
`robot_s0_impedance.yaml` sets conservative impedance gains for validation.
Robotiq uses requested binary position (0=open/255=close), not measured width;
it uses the collection settings speed32/force0, never auto-calibrates, and clears
GoTo on pedal release/stale policy input without automatically opening.

On a fresh source snapshot, after applying `compat.patch`:

```bash
cp policy_watchdog.h src/serl_franka_controllers/include/serl_franka_controllers/
patch -d src -p1 < runtime_guard.patch
~/.pixi/bin/pixi run build
```

Both patches and the header are already installed on robot-s0. `run_client.sh`
runs `prepare_realtime.py` before live arming: it demotes inherited real-time
background threads in this user's verified franka_control process while retaining
FIFO99 on its main FCI thread. Motion controllers must be inactive for this step.
The original collection environment remains unchanged.

## Full-speed attempt after Home

On 2026-09-11 the operator requested Home followed by one full-speed, full-length
trial. The latest **remote** `conf/full100b.conf` (not the stale Mac/full50b Home)
matches ep046's saved configuration. Its Home is:

`[-0.03351456, -0.07893287, 0.45646879, -1.56279087, 0.01271964, 1.89761698, 1.63824415]`

ROS relinquished the FCI; the existing `bin/pnp7_teleop home conf/full100b.conf`
completed in 1.925 seconds. ROS then verified the joints near the saved target.
Home requires the pedal released at startup; it is **not** hold-to-run.

The new explicit `--profile full` uses asynchronous inference and native model
deltas at 30 Hz, without the smoke profile's amplitude scaling or 5 cm envelope.
It retains the demonstrated workspace, action outlier rejection, tracking guards,
camera/state freshness, controller watchdog, and foot-pedal gating. New chunks
replace the unexecuted old suffix; this is ordinary delayed-feedback control,
not RTC or latency-compensated inference. Five Python tests pass.

**The requested 40-second trial did not complete.**
`validation/full_40s_20260911.json` records:

- Pedal press received; four commands at intervals 33.52, 33.05, 35.14 ms.
- At active time 0.343 s, the fifth candidate reached **16.27 mm** target error,
  exceeding the retained **15 mm** client guard. Rotation error was 0.0208 rad.
- Stopped automatically; final robot idle; no current/last-motion errors;
  gripper fault0; no cleanup errors. Net measured movement approximately 0.214 mm.
- No automatic retry or safety-threshold increase was performed.

Controller gains remain the conservative smoke-test values (translation1000,
damping63, 5 mm error clip). SERL target filtering is still 0.005 per 1 kHz step
(approximately 0.20 s time constant). Full-speed tracking therefore needs further
controller-response investigation and supervised tuning, not a claim of policy
task failure or a blind increase to the tracking-error threshold.

References: [RoboStack Pixi setup](https://robostack.github.io/GettingStarted.html),
[SERL controller](https://github.com/rail-berkeley/serl_franka_controllers).
