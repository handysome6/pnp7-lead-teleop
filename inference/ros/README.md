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

`validate` selects live/full, native learned deltas, maximum 40 seconds
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
workspace limits, controller gains/filter and hardware collision reflexes are
unchanged. The larger lag allowance is not proof of successful tracking.
Logs: `validation/live_full_<timestamp>_<pid>.json`, including verified Home and
tracking limits. `pixi run shadow` remains a zero-motion observation diagnostic;
it does **not** Home and is not the live-validation entry point.

The sections below describe earlier tests and historical settings.

### Two-step inference prefixes (current validation behavior)

The full client still requests predictions asynchronously, but executes only
indices **0 and 1** of each selected chunk, separated by a 30 Hz control tick.
It then waits for a strictly newer prediction and discards indices 2..9. A new
prediction does not interrupt the selected two-step prefix; after that prefix,
the newest available chunk is selected without queuing old predictions.

Waiting does not accumulate deltas, repeat actions, publish pose targets, or
refresh gripper commands. Camera/state/pose tracking checks continue; existing
250 ms command watchdogs and 400 ms observation-age checks remain effective.
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
