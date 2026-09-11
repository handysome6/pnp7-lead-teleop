# Collection GUI

An operator panel over this repo's own collection path: camera previews, arm
restore, session and episode control, teleop-only mode, and F3-gated recording.

```bash
python -m gui.selftest            # 59 checks, no hardware
python -m gui.app --mock          # the interface, against a fake rig

.venv/bin/python -m gui.app       # the real thing, on the robot PC
pixi run python -m gui.app       # robot-s0, using its pixi environment
```

Open `http://127.0.0.1:8770` on the controller. From a laptop, forward the
loopback port with `ssh -N -L 8770:127.0.0.1:8770 robot-s0`, then open the same
URL locally. The GUI and its Python subprocesses use the same interpreter.
An idle page does not validate camera access, FCI connectivity, or foot-brake
press/release behavior; those must be ready before opening a real session.

Website camera previews use MJPEG capped at 5 fps per camera, skipping
unchanged JPEGs to save bandwidth over remote connections. Recorded camera
frames remain 640×480 at 30 fps with JPEG quality 90.

It supervises `bin/pnp7_teleop` and `collect/record_cameras.py` — the same
processes `scripts/collect_episode.sh` runs, in the same order — with the
operator deciding where each take starts and ends. Nothing touches the robot
until a session is opened.

## Configuration choices

The default menu contains **标准遥操** (`full100b.conf`, 1:1 mapping) and
**半幅遥操** (`full50b.conf`, 0.5 gain), both with binary gripper control.
Select **显示关节调试配置** to expose `j6only.conf` and `j7only.conf`
(0.25 gain, gripper disabled). Configuration choices are locked while a
session is open or a Home save is in progress. Saving Home changes only the
selected file.

`demo.conf` remains available to `demo.sh` but is excluded from the GUI menu.
The old-posture configs `firstlive`, `full25`, `full50`, `full50g`, `j34`,
`pnp7_teleop`, and `wrist` have been removed; they retained earlier joint
signs. The collection shell script now defaults to `full50b.conf`.
New, unclassified configs appear only under the debug toggle. Historical
snapshots in `known_good/` and `archive/` are outside the GUI config directory.

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

### Save the current pose as Home

Select a config and click **将当前位置保存为 Home** in the Robot panel. This
works while idle, between takes in a collect session, or in a teleop-only
session with F3 released. During a session it writes only that session's
selected config. Stop motion and leave guiding mode first. Recording, held
F3, and mock/dry-run sessions cannot save a real pose.

In teleop-only mode, both Save Home and **Restore start joints** check fresh
bridge status with F3 released, stop the bridge cleanly to release FCI, then
read the measured pose or move to the configured Home. The physical F3 state
is checked again before the operation. The camera preview stays open. On
success or a preflight refusal, the bridge resumes with a new clutch origin;
if F3 is held during restart, release and press it again to enable mapping.
A control fault stops the session instead of automatically resuming it.
Restore always uses the selected config's latest `home_qpos`, including a
pose just saved in that session.

The button calls `pnp7_teleop read-home <config>` to read the actual seven
joint angles through FCI. This mode does not command motion or open the
gripper, GELLO, or cameras. It refuses a moving robot or a pose outside the
joint envelope used by `home`. Connection errors leave the config unchanged.

The GUI updates `home_qpos` directly in `conf/<selected>.conf`, preserving
the other parameters and comments, and shows the saved angles in radians.
The previous file is saved as `<selected>.conf.home.bak`; the updated config
is replaced atomically. A concurrent edit during capture aborts the save.
The next Restore uses the new pose; no compilation is needed after saving.
`calibration.json` is not changed, so regenerating a config from calibration
later will replace this manually saved Home.

Offline coverage: `pixi run python -m unittest gui.test_home` exercises config
writes, backups, rejected states, and the HTTP action using a temporary config
and a stand-in state reader. It never reads or moves the actual robot.

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

### Collision-reflex discard during collection

A take aborted solely by `cartesian_reflex` or `joint_reflex` is automatically
discarded: the bridge and recorder stop, the failed take's directory is removed,
and the collect session returns to READY with its successful-take count intact.
Small diagnostic logs are retained under `/tmp/pnp7_discarded_<episode>_*`;
the SDK's diagnostic CSV remains at the path reported by the bridge. The GUI
shows the discard reason and waits for the operator to start the next take.
Stop & keep also checks the bridge exit code, so clicking it just before the
health poll cannot accidentally keep a reflex-aborted take.

This does not clear the robot's protection fault or start another motion.
Release F3, remove the contact, and acknowledge recovery in Desk if required
before starting the next take. Discontinuities, communication faults, mixed
error lists, cleanup failures, and teleop-only faults still latch ERROR.

### Resume counts and numbering after restarting the GUI

The GUI reads the selected dataset directory and prefix from disk on startup
and when opening a session. Saved counts require a nonempty `episode.csv`,
positive `episode_meta.json` frame count, and no `failure.json`; unfinished
or failed directories are shown separately. Counts describe saved takes,
not a guarantee that every dataset quality check passed.

The next ID starts after the highest existing numeric ID for that prefix,
without filling gaps. Before recording, a locked, atomic
`.pnp7_episode_sequence.json` reservation advances the ID. Once reserved,
IDs are not reused after a discard or process restart. Existing directories
and files are never overwritten. Use the same dataset directory and prefix
to continue the same collection.

The GUI builder uses `external` when present, otherwise the available non-wrist
camera (such as `cam2022`), keeping actual camera names in the dataset. A failed
build preserves raw data and surfaces the failure instead of claiming a saved
take with zero frames.

## Operator flow

| Step | Notes |
|---|---|
| Open collect / teleop only | starts the viewfinder; successful teleop writes no dataset CSV; control faults retain diagnostic CSVs in `/tmp` |
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

robot-s0 uses `gripper_type=robotiq` in `full100b` and `full50b`. The driver
connects to the persistent FTDI RS-485 by-id path at 115200 8N1, slave 9. It
supports the shared 2F-85/2F-140 protocol without guessing which model is fitted.
The GELLO trigger follows the existing calibrated open/closed ticks and binary
hysteresis. `robotiq_speed=32` and `robotiq_force=0` are raw register values;
force 0 means minimum gripping force, not zero physical force.

Startup reads status and seeds the target from the measured raw position. It
does not reset or activate the gripper. A pre-existing communication timeout
can be cleared by preserving rACT and sending Stop, without a reset/activation
edge. A gripper that is still unready must be explicitly initialized first:

```bash
# Read status / check readiness; does not open or close the fingers.
./bin/pnp7_teleop robotiq-check conf/full100b.conf
# Explicit activation: automatically moves the fingers to calibrate their travel.
./bin/pnp7_teleop robotiq-init conf/full100b.conf
# GELLO + foot-brake test, 30 seconds, without connecting to the Franka arm.
./bin/pnp7_teleop robotiq-test conf/full100b.conf 30
```

A separate SCHED_OTHER worker performs all serial I/O and sends a heartbeat
at least every 100 ms while healthy. Release the foot brake to send Stop;
this holds the current position and does not open or reset the fingers.
No movement request is sent until the pedal has first been released and then
pressed. The worker checks status continuously. Serial timeout, CRC error or
an unready/fault status latches an error; the arm decelerates to a stop and the
GUI displays the reason. Shutdown attempts Stop, with bounded serial waits,
and joins the worker. A disconnected cable cannot deliver a stop command.

The GUI shows raw position/request (0=open, 255=closed). `teleop.csv` and
`episode.csv` preserve `gripper_position_raw`, `gripper_requested_raw` and
`gripper_fault`. Uncalibrated metre-valued width/target remain -1. The existing
metre-based LeRobot exporter explicitly rejects Robotiq rows, instead of
silently exporting false zero-width measurements. Model/width calibration or
an explicitly normalized export schema is required before exporting them.

Protocol reference: [Robotiq 2F control manual](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM-OMRON_InstructionManual_HTML5_20190503.zip/2F-85_2F-140_TM-OMRON_InstructionManual_HTML5/Content/4.%20Control.htm).
Offline transport tests use a pseudo-terminal and exercise fragmented replies,
startup without movement, closing/reopening, pedal release, shutdown, CRC,
timeout, reported faults and explicit activation (`diag/test_robotiq.py`).

If startup prints `preflight ok` and then fails while connecting the gripper,
the arm's FCI connection succeeded. The Franka Hand has a separate connection
on port 1338; libfranka's generic connection-refused message can incorrectly
suggest that the arm's FCI mode is disabled. The bridge now identifies this
stage explicitly. Check the Hand connection and Desk end-effector configuration.
Use `gripper_type=robotiq` for Robotiq, or `gripper_enabled=0` for arm-only operation;
a failed required gripper connection is not silently ignored.

Runtime control faults are shown in the page's error panel and printed to the
GUI process's stderr, including exit code, original Franka error and log path.
A failed session enters ERROR with streaming disabled and requires an explicit
close before another session can be opened; it never restarts motion itself.
Teleop-only bridge logs use unique `/tmp/pnp7_bridge_<timestamp>.log` names.
Both teleop and Home explicitly enable libfranka's final command rate limiter
(velocity, acceleration and jerk), with the existing 100 Hz SDK filter. The
application's conservative limits still apply upstream. This does not provide
realtime scheduling or eliminate failures under excessive packet loss; the
temporary `kIgnore` kernel trial remains in place. Teleop and Home now refuse
to start unless the FCI thread actually obtained `SCHED_FIFO` priority. On
robot-s0, `/etc/security/limits.d/99-franka-liushuai.conf` grants liushuai rtprio
99; new login sessions inherit it. An already-running GUI needs its rtprio
resource limit updated or a fresh login before restarting it. The bridge prints
`FCI scheduling: SCHED_FIFO priority=99` during preflight. GELLO, foot-brake,
status and gripper worker threads explicitly use `SCHED_OTHER` so they do not
inherit FIFO 99 from the control thread.
On a Franka control exception, the bridge saves the library's last 5000 states
and commands to `/tmp/pnp7_franka_error_<timestamp>.csv` outside the control
callback. It preserves 17-digit precision, desired acceleration, robot mode
and error flags. Each SDK row pairs command n with state n+1; the default 50
frames can contain only the end of reflex braking and miss its onset.
The error text also reports the first recorded fault's success rate separately;
libfranka's final success rate may already have recovered during reflex braking.
The raw callback output and periods are also saved: a failed recording retains
its `teleop.csv`, while teleop-only keeps its last 5000 samples in
`/tmp/pnp7_control_error_<timestamp>.csv`. These logs allow comparison of the
generator output with the conditioned wire command and received robot state.
Failed recordings retain their raw files and `failure.json`; they are not
automatically accepted into the dataset or silently deleted. Ordinary explicit
discard of a healthy take is unchanged.

Run the offline fault regressions with `pixi run python -m unittest gui.test_faults`.
Run the synthetic FCI timing-gap and diagnostic-precision regression with
`pixi run bash diag/test_fci_continuity.sh` from a login with rtprio 99 permission.
It also verifies real scheduling permissions and worker priority isolation.
It opens no hardware and does not
replace an on-robot timing validation.

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

### Browser transport

The page fetches finite `/preview/<camera>` JPEG responses at up to 5 fps
per camera. Each camera has one request at a time, with a 3 s timeout; removed
previews abort their requests. `/stream/<camera>` remains available for legacy
MJPEG clients. State polling has one in-flight request and a 5 s timeout, so
a blocked browser connection cannot accumulate unlimited polls.

The page suppresses duplicate pending button commands. Start, open-session,
Home-save and Home-restore requests carry the timestamp from their last
server state response; requests older than 5 s are rejected on receipt and
again before execution. Network retries never replay a robot action.

### FCI callback deadlines and completion

The September 10 ep027/ep028 traces show host callback-entry gaps of 6.590 ms
and 6.769 ms while the reported robot period was still 1 ms; the following
callback had a 5 ms robot-time gap. Both faults then reported acceleration
discontinuity. A subsequent local-generator completion caused libfranka's
finishMotion path to label the already-aborted move as “still moving”. See
[libfranka 0.21.3 finishMotion](https://github.com/frankarobotics/libfranka/blob/0.21.3/src/robot_impl.cpp).

The realtime callback now reads GELLO with try-lock and retains the previous
coherent sample on contention. Freshness uses that sample's timestamp, not
a newer timestamp from a sample it could not read. Callback-entry gaps above
3 ms, robot periods above 3 ms, or callback work above 1 ms cancel the move
through libfranka's error/StopMove path instead of returning a late position
command. These checks stop and report a delay; they cannot eliminate kernel,
network, or firmware scheduling delays. They never recover a robot fault.

Teleop and Home finish only after the local generator is stopped, there is no
robot error, and accepted position/velocity/acceleration plus measured velocity
have remained near rest for 100 ms. The finish guard resets on an error or
a timing gap. Fault data remain excluded from saved counts.

Offline regression and optional trace replay (no hardware is opened):
`pixi run bash diag/test_control_stop.sh episodes/ep027/teleop.csv episodes/ep028/teleop.csv`.
