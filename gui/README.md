# Collection GUI

An operator panel for single-arm RLinf data collection: camera previews, arm
restore, session and episode control, teleop-only mode, and F3-gated recording
that lands in LeRobot format.

```bash
python -m gui.selftest          # 29 checks, no hardware, no RLinf
python -m gui.app --mock        # the interface, against a fake rig
```

## Why it is shaped this way

**The GUI runs inside the RLinf worker.** Not beside it, not as a supervisor
that shells out to `collect_data.sh`. That is forced, not chosen:

- Every device admits exactly one owner. `RealSenseCamera.__init__` calls
  `pipeline.start()`, franky holds the FCI control connection, and the GELLO
  leader is a serial port. A second process could only ever watch.
- RLinf's realworld stack cannot run without Ray. `FrankaEnv._setup_hardware`
  asserts `isinstance(self.hardware_info, FrankaHWInfo)`, so `worker_info=None`
  fails outright, and `FrankyController.launch_controller` spawns the arm
  controller as a Ray actor.

So there is one process, it holds everything, and the GUI lives in it.
`gui.collect_gui` is `examples/embodiment/collect_real_data.py` with the fixed
`while success_cnt < target` loop replaced by an operator.

**A browser, not a native window.** This repo already documents the cost of the
alternative: `scripts/collect_episode.sh` has to export `DISPLAY` and
`XAUTHORITY=/run/user/1000/gdm/Xauthority` to get a cv2 window up, and RLinf's
own `VideoPlayer` silently disables itself when `DISPLAY` is unset. A browser
also puts the preview on a laptop next to the rig rather than on the robot PC's
monitor. The server is `http.server` plus `cv2.imencode` — no web framework, so
nothing new is installed into the environment that has to keep working.

## Nothing in RLinf is modified

Three seams, all public by construction:

| Seam | Used for |
|---|---|
| `CollectEpisode` honours `pre_record` / `record_reset` / `segment_advance` from `info` | F3 clipping (`gui/episode_control.py`) |
| `FrankaEnv.camera_player` is a plain attribute assigned after the cameras open | previews, by swapping in a capture shim |
| `FrankyController.reset_joint` | restoring the start pose |

That matters because the single-arm Franky/GELLO branch is headed for an
upstream RLinf PR. A GUI-driven collector is rig-specific, so it lives here and
the upstream diff stays about the robot.

## F3 does two jobs

It is already the GELLO deadman — hold-to-enable absolute streaming. The GUI
makes it the recording gate too:

```
pre_record = not (episode_open and gello_deadman_held)
```

`GelloJointIntervention` already publishes `info["gello_deadman_held"]` every
step, so nothing opens the pedal a second time. `CollectEpisode` then drops
released-pedal frames before they reach the buffer — the same thing
`build_episode.py` does for the old C++ pipeline.

Each held interval gets its own `segment_id`, written per frame into the LeRobot
episode. This is worth knowing when training: the arm pose *is* continuous
across a pedal gap — the stream holds at the measured position while F3 is up,
and re-engagement is refused unless GELLO and Franka agree within
`startup_max_error` — but wall-clock time is not. The seams are marked so a
training pipeline can honour them or ignore them, rather than having to guess
where they were.

## What the shipped config does today

`realworld_collect_data_gello_franky.yaml` has no operator-driven episode
boundary, and this is the gap the GUI closes. As it stands:

- no `keyboard_reward_wrapper`, so `manual_done` is never set;
- `max_episode_steps: 300`, so every take is a fixed 30 s at 10 Hz;
- `only_success: True`, and without a reward model the reward is TCP proximity
  to a fixed `target_ee_pose` — a SERL-style reaching check, unrelated to a
  pick-and-place demonstration.

So takes would be cut at 30 seconds and then kept or dropped by a proximity test
that has nothing to do with the task. The GUI sets `max_episode_steps: null` and
`manual_episode_control_only: true` — RLinf's own spelling for "an external
wrapper owns the episode end" — and passes `only_success=False`, because an
episode the operator chose to keep is the success signal.

## Running it on the rig

```bash
cd ~/workspace/andyls/RLinf
export EMBODIED_PATH=$PWD/examples/embodiment
export PYTHONPATH=$PWD:$HOME/workspace/andyls/pnp7-lead-teleop:$PYTHONPATH

python -m gui.collect_gui \
    --config-path $EMBODIED_PATH/config \
    --config-name realworld_collect_data_gello_franky \
    runner.logger.log_path=$PWD/logs/gui
```

Binds `127.0.0.1:8770`. To drive it from a laptop pass `gui.host=0.0.0.0` and
use the token it prints — the page commands a 7-DoF arm, so reaching it from
another machine should take a secret and not just the right IP.

Nothing touches the robot until you press **Open**. The env, and with it the FCI
connection and the RealSense pipelines, is built then.

## Operator flow

| Step | Notes |
|---|---|
| Open collect / teleop only | teleop-only constructs no writer at all, so nothing *can* be written |
| Restore start joints | refused while F3 is held or an episode is open — the move would otherwise be recorded as demonstration |
| Start episode | `s` |
| Hold F3 | only these frames are kept; release and re-press opens a new segment |
| Stop & keep / Discard | `e` / `x` |
| Finalize LeRobot metadata | writes `info.json` / `stats.json`; until it runs the shard is on disk but not loadable |

## Known limitations

- **No preview before a session.** The env claims the RealSense pipelines
  exclusively, and releasing them cleanly enough for pyrealsense2 to re-open is
  not reliable. Open a teleop-only session to frame the scene.
- **The config *file* is fixed at launch.** Hydra composes at process start, so
  switching files means relaunching. The GUI varies parameters within the
  launched config, which is what actually changes between takes.
- **Preview rate follows the env**, not `collect/record_cameras.py`. RLinf's
  `BaseCamera._capture_frames` sleeps `1/fps` and *then* reads, so the effective
  rate sits below the nominal `CameraInfo.fps` (default 15).

## Files

```
session.py         state machine, backend contract, control loop
episode_control.py the gym wrapper that turns F3 into pre_record/segment_id
rlinf_backend.py   the real rig, inside the Ray worker
mock.py            a fake rig, for developing off the robot PC
server.py          stdlib HTTP: JSON state, MJPEG previews, command queue
static/index.html  the panel
app.py             standalone mock entrypoint
collect_gui.py     hydra + Ray entrypoint for the rig
selftest.py        29 offline checks, including the F3 clipping arithmetic
```
