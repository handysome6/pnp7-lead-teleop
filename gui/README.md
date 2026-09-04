# Collection GUI

An operator panel over this repo's own collection path: camera previews, arm
restore, session and episode control, teleop-only mode, and F3-gated recording.

```bash
python -m gui.selftest            # 59 checks, no hardware
python -m gui.app --mock          # the interface, against a fake rig

.venv/bin/python -m gui.app       # the real thing, on the robot PC
```

It supervises `bin/pnp7_teleop` and `collect/record_cameras.py` — the same
processes `scripts/collect_episode.sh` runs, in the same order — with the
operator deciding where each take starts and ends. Nothing touches the robot
until a session is opened.

## Why supervision, not integration

Everything here is already a separate process, so running them is the natural
fit rather than a workaround. Three properties of the bridge shape the whole
design, and all three are easy to get wrong:

| Property | Consequence |
|---|---|
| The log lives in RAM until the process exits — `rows(duration * 1100)` allocated up front, `writeLog` called once at the end | **Never `SIGKILL`.** Stopping is always SIGINT, which decelerates the arm to rest, writes the CSV, and exits. A kill destroys the entire take. |
| SIGINT and a completed run both exit 0 | Completion is judged by row count, never by exit status |
| Nothing in the bridge ever flushes stdout | Over a pipe it is 4 KB block-buffered, so `CONTROL_READY` may never arrive. Readiness is the **status file** appearing instead |

That last one is lucky rather than clever: `runRobot` already publishes a JSON
status file at 10 Hz via write-tmp-then-rename, and deletes it on a clean exit.
So its appearance is the readiness signal, its content is the live telemetry,
and its disappearance is the crash signal. `collect/view_cameras.py --status`
has been reading the same file all along.

## Clipping happens downstream, and that is the point

There is no clipping code in this GUI. `deadman` is a column in `teleop.csv`,
and `build_episode.py` drops the released rows when it joins the streams. So
what counts as demonstration stays a pure function of data already on disk:
change your mind about the policy and re-run `build_episode.py`, months later,
with the robot switched off.

The self-test demonstrates this rather than asserting it — a stubbed take with
the pedal released in the middle comes out of the *real* `build_episode.py` with
the idle frames dropped and counted in `episode_meta.json`.

## The C++ change

One new mode, `pnp7_teleop home <config>`, because relative joint mapping has no
notion of an absolute pose: after a session the arm is wherever the operator
left it. It drives to `home_qpos` on a quintic profile — zero velocity *and*
acceleration at both ends, which matters because libfranka rejects a motion that
finishes moving. A SIGINT decays the time-scaling to zero over 0.3 s rather than
stopping dead, for the same reason.

The duration comes from the config's own `max_joint_velocity` /
`max_joint_acceleration`, inverting the quintic's peak coefficients (1.875·d/T
and 5.7735·d/T²). That is deliberately conservative: at the stock 0.3 rad/s a
3.5 rad move takes ~22 s. Raise `max_joint_velocity` if that is tedious — the
compiled ceiling of 0.60 rad/s still applies.

`home_qpos` comes from `calibration.json`'s `franka_rest_pose`, emitted by
`calib/make_teleop_config.py`. **Configs generated before this change do not
have it, and `home` refuses to run without it** — regenerate, rather than
guessing a pose nobody chose.

`runDry` also gained a status publisher, so a dry session shows the same
telemetry as a live one instead of looking like a bridge that failed to start.

## Camera recorder changes

Two additive flags; without them the behaviour is unchanged bit for bit, so
`collect_episode.sh` is unaffected.

- `--preview-dir DIR` — publish the latest frame per camera as `<role>.jpg` at
  ~10 Hz, written to a temp name and renamed into place. Taps frames already
  captured, so there is no extra camera load. This is how the browser sees the
  scene: every RealSense pipeline is exclusive, so nothing else can open them.
- `--no-write` — hold the cameras and record nothing. A viewfinder for framing
  the scene between takes.

## Operator flow

| Step | Notes |
|---|---|
| Open collect / teleop only | starts the viewfinder; teleop runs the bridge with no log path, so nothing *can* be written |
| Restore start joints | refused while an episode is open, and refused by the bridge itself if F3 is held |
| Start episode | `s` — viewfinder stops, recorder starts, then the bridge once the cameras are ready |
| Hold F3 | only these frames survive `build_episode.py` |
| Stop & keep / Discard | `e` / `x` — SIGINT to the bridge, then build + validate. Discard actually deletes the directory |
| Re-validate all episodes | runs `validate_episode.py` across the directory |

**Release F3 before starting a take.** A pedal already held when the bridge
starts is latched out until released once, and the bridge reports that
identically to not pressing at all — the arm simply will not move and nothing
says why.

## Known limitations

- **The arm is dead between takes.** The bridge only runs during an episode, so
  repositioning outside one means `home`. This matches what
  `collect_episode.sh` has always done; a session-long bridge would cost ~1 GB
  of RAM for half an hour and lose everything on a crash.
- **Live counters are seconds, not frames.** The bridge publishes no frame
  count, so the panel integrates pedal state at 10 Hz. `episode_meta.json`
  replaces this with the exact number the moment the take is built.
- **Teleop sittings are bounded** (default 600 s) because the bridge sizes its
  RAM buffer from the duration. It is not auto-restarted: a restart while F3 was
  held would silently latch the pedal out.
- **Event camera is not covered** — equivalent to `EVENTS=0`. `event_camera/` is
  a separate subsystem whose container runs a manual copy of the code.

## LeRobot

`collect/export_lerobot.py` converts built episodes, offline, in the RLinf
environment (this repo's `.venv` has neither `lerobot` nor `torch`):

```bash
export PYTHONPATH=$HOME/workspace/andyls/RLinf:$PYTHONPATH
$RLINF_PYTHON collect/export_lerobot.py episodes/ \
    --out ~/datasets/pnp7_lerobot --task "pick the block and place it in the bin"
```

It reuses RLinf's `LeRobotDatasetWriter` rather than reimplementing the format —
that class depends only on a compat shim and a logger, so it lifts out of the
framework cleanly. `state` and `actions` are both 8-dimensional: seven joints
plus the gripper, measured for state and commanded (`q_command`) for actions,
because the roadmap is emphatic that the training action is never the raw lead
arm.

Each F3 interval is recorded in a per-frame `segment_id`; `--split-segments`
emits them as separate LeRobot episodes instead, if you would rather the seams
be impossible to cross. `--dry-run` reports what would be written.

## Files

```
session.py         state machine, backend contract, control loop
legacy_backend.py  supervises the bridge and the recorder
mock.py            a fake rig, for working off the robot PC
server.py          stdlib HTTP: JSON state, MJPEG previews, command queue
static/index.html  the panel
app.py             entrypoint, --mock or real
selftest.py        59 offline checks
stubs/             test doubles for the bridge and the camera recorder
```

The stubs are what let the self-test run the *real* `build_episode.py` and
`validate_episode.py` on a laptop: only the two hardware-facing processes are
stand-ins, so the join, the clipping and the validation are all exercised for
real.
