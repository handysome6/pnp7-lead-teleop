# PNP-7 lead arm -> Franka teleoperation

Implements V1 of `pnp7_roadmap.md`: relative joint-space teleoperation from the
PNP-7 lead arm to the Franka, for VLA/imitation-learning data collection.

## Hardware as discovered

Nothing about this bus was documented, so it was identified by probing.

| | |
|---|---|
| Lead arm bus | 8x Dynamixel XL330, Protocol 2.0, half-duplex behind an FT232H |
| Lead arm device | `/dev/pnp7_lead` (udev symlink, stable across replug) |
| Joints | IDs 1-7 = J1..J7, XL330-M288-T (model 1200) |
| Gripper trigger | ID 8, XL330-M077-T (model 1190) |
| Firmware | 52 on all servos |
| Franka | Panda @ `172.16.0.2` via `enp4s0` (172.16.0.1/24), libfranka 0.15.0 |
| Cameras | 2x RealSense D435i - `213622078826` external, `233622071437` wrist |
| Kernel | 5.15.197-rt91 PREEMPT_RT |

The lead arm is a passive input device: torque is disabled on every servo so
the operator backdrives it by hand. Every tool here except `set_baud.py` and
`tune_bus.py` is strictly read-only on the servo bus, and all of them refuse to
run if they find torque enabled.

## Changes made to the hardware

Two persistent EEPROM settings were changed on the lead-arm servos, because the
factory defaults capped whole-arm sampling at ~31 Hz:

| Register | Was | Now | Effect |
|---|---|---|---|
| Baud rate (addr 8) | 57600 | 1000000 | wire time 25 ms -> 1.4 ms |
| Return delay (addr 9) | 250 (500 us) | 0 | removes 8 x 500 us per cycle |

Measured result: **31 Hz -> 500 Hz** position-only, **21 Hz -> 420 Hz** with
velocity, zero failed frames over 1500 reads.

To revert:

```bash
python tune_bus.py --raw 250 --yes
python set_baud.py --from-baud 1000000 --to 57600 --yes
```

A udev rule (`/etc/udev/rules.d/99-pnp7-lead.rules`) pins the adapter to
`/dev/pnp7_lead` and sets the ftdi_sio `latency_timer` to 1 ms (the 16 ms
default alone cost ~16 ms per cycle). `franka` was added to the `dialout` group.

The same rule pins the dead-man button to `/dev/pnp7_deadman`. This was learned
the hard way: both devices were moved onto a USB hub, and while the lead arm
survived (matched by serial), the dead-man moved from `event11` to `event6`
while every config still named `event11`. By then `event11` had become the
Dell optical mouse. That failure is fail-safe -- a mouse reports `BTN_LEFT`
(0x110), never `BTN_0` (0x100), so nothing could have enabled motion -- but the
session would simply never have engaged, with no obvious reason.

Never put a bare `/dev/input/eventN` in a config. `make_teleop_config.py`
prefers the symlink.

### The foot switch that replaced the SpaceMouse

2026-08-23: the SpaceMouse was removed and a USB foot switch put in its place.
It is a nameless STM32 composite HID device, `0483:5750`, with **no
manufacturer, product or serial string at all** -- `lsusb` only shows what
`usb.ids` guesses from the VID:PID (it claims "LED badge", which it is not),
and the kernel calls it `HID 0483:5750`. It presents three interfaces: 00 is a
vendor-defined configuration channel, 01 a keyboard, 02 a mouse. The pedal is
on interface 01 and emits **`KEY_F3`**; interface 02 is unused here.
`find_button.py` is what identified this -- nothing about the device announces
it, so it has to be pressed while something watches every `event*` node.

**It ships in one-shot mode and has to be reprogrammed.** Out of the box it
emitted a single ~32 ms `KEY_F3` pulse however long the pedal was physically
held -- press and release 32 ms apart, no auto-repeat, nothing for the
remaining three seconds of a three-second hold. That cannot drive a
hold-to-enable dead-man at all, and no amount of software can work around it:
the device simply never reports that the pedal is still down. The vendor's
WebHID page switches it to normal-key mode, after which a hold looks the way it
should:

```
18:01:56.950  KEY_F3 down
              KEY_F3 repeat x ~80, every 40 ms      <- kernel auto-repeat
18:02:00.142  KEY_F3 up                             ~3.2 s
```

If the dead-man ever starts behaving like a toggle again, or `check_ready.py`
passes while the arm still refuses to move, suspect the switch has been reset
to one-shot mode. `grab_button.py` shows the held time per press and settles it
in one step.

Reprogramming needs the vendor page to reach interface 00 over WebHID, and that
needs three things:

- Chrome. Firefox has no WebHID at all -- `navigator.hid` is undefined and the
  page's button does nothing.
- The page open **on the machine the switch is plugged into**.
- Read/write on the interface-00 hidraw node. `/dev/hidraw*` is `root:root
  0600` by default, so Chrome (running as `franka`) cannot enumerate it and the
  device chooser comes up empty -- which looks like the switch is not
  connected. The udev rule opens up interface 00 only, as
  `/dev/pnp7_deadman_cfg`; interfaces 01 and 02 stay root-only, because those
  carry real keystrokes and no web page should be able to read them.

Two consequences for the udev rule:

- There is no serial to match on, only VID:PID plus the interface number.
  These have to be matched as `ENV{ID_VENDOR_ID}` / `ENV{ID_MODEL_ID}` /
  `ENV{ID_USB_INTERFACE_NUM}`, **not** as `ATTRS{...}`. Every `ATTRS` key in
  one rule must match on the *same* parent device, and `idVendor` lives on the
  USB device while `bInterfaceNumber` lives on the interface below it, so the
  `ATTRS` spelling silently matches nothing and no symlink appears.
- `KEY_F3` is an ordinary keystroke and some application on the desktop holds a
  global binding for it, so every press raised a window on top of the session.
  The rule sets `ENV{LIBINPUT_IGNORE_DEVICE}="1"` on both interfaces, which is
  enough on X11 as well as Wayland: `xf86-input-libinput` honours the property
  and refuses to create the device, so the X server drops it and it no longer
  appears in `xinput list`. The `xorg.conf.d` `Option "Ignore"` spelling was
  tried and dropped -- `xorg.conf.d` is only parsed when the X server starts,
  so it does nothing until the next login.

The bridge additionally takes the device exclusively with `EVIOCGRAB`
(`deadman_grab=1`, the default). The udev property covers the desktop; the grab
covers any other evdev reader for the duration of a session. The kernel drops
the grab when the fd closes, including on a crash, so the button cannot be left
captured. `grab_button.py` does the same thing standalone, for checking the
takeover without starting the bridge.

## Teleoperation bridge

`pnp7_teleop.cpp` implements roadmap V1. Two threads, per roadmap section 12:
the lead arm is sampled on its own thread and published as an atomic snapshot;
the 1 kHz FCI callback only reads that snapshot, runs the safety chain, and
returns. No USB traffic, allocation, or logging I/O happens inside the callback.

Dead-man is hold-to-enable. A button already held at startup is ignored until
it has been released once. Which key counts is `deadman_key` in the config
(`KEY_F3` for the current button; `BTN_0` was the SpaceMouse). The compiled
default stays `BTN_0` on purpose: the new button does not claim `BTN_0`, so a
config written before the swap fails loudly at startup rather than silently
assuming which button is attached. Auto-repeat (`value == 2`) is ignored --
the foot switch is a keyboard, so the kernel emits a repeat every 40 ms for as
long as the pedal is down, and treating one as a fresh press would defeat the
release-first latch. The SpaceMouse never did this, so the case did not exist
before the swap.

A fatal read error on the dead-man fd latches `pressed()` to false for the rest
of the run and disengages with reason `deadman_unplugged`. The kernel does
release held keys when a device is removed, but if that release were ever
missed the last value read would be a press that nothing can clear, and the arm
would stay enabled by a switch that is no longer attached.

Safety chain, applied in this order every cycle:

1. relative mapping `q = q_origin + sign * scale * (lead - lead_origin)`
2. per-joint session clamp (`max_session_delta`)
3. **joint-limit clamp on the desired position** - before rate limiting, never
   after; clamping the output instead lets the clamp emit a step of arbitrary
   size that bypasses the velocity and acceleration limits
4. low-pass filter (`lowpass_hz`), with the filter state clamped so it cannot
   wind up outside the envelope
5. velocity limit, capped additionally by the discrete-exact braking bound
   `sqrt(2*a*d + (a*dt)^2) - a*dt`, so a joint never enters the target faster
   than it can stop
6. acceleration limit

Releasing the dead-man, a stale lead arm, or `SIGINT` all route to `hold()`,
which decays velocity to zero and freezes. A session only reports
`motion_finished` once every joint is actually at rest, because libfranka
rejects a motion that ends with non-zero velocity.

Config values are validated against compiled ceilings and can only ever be more
conservative than them.

### Build and run

```bash
./build.sh
./bin/pnp7_teleop selftest pnp7_teleop.conf          # offline, 10 checks
./bin/pnp7_teleop dry      pnp7_teleop.conf 30 dry.csv   # hardware, no robot
./bin/pnp7_teleop robot    pnp7_teleop.conf 60 run.csv   # live
```

`dry` runs the entire pipeline - lead arm, dead-man, clutch, safety chain -
against a simulated rest pose, so mapping and directions can be confirmed with
the robot untouched. Always run it before `robot`.

## Data collection

```bash
DURATION=60 CONF=full50g.conf ./collect_episode.sh episodes/ep001
```

Starts both cameras, waits for `CAMERAS_READY` (auto-exposure needs to settle
before frames are worth training on), runs the bridge, then joins everything.
`MODE=dry` exercises the whole path without commanding the robot.

Output per episode:

```
episodes/ep001/
  cam_external/000000.jpg ...     cam_external_index.csv
  cam_wrist/000000.jpg ...        cam_wrist_index.csv
  teleop.csv          1 kHz bridge log, 65 columns
  episode.csv         joined dataset at camera rate
  episode_meta.json   frame count, rate, alignment statistics
  config.conf         the exact config used
  calibration.json    the exact calibration used
```

### Synchronisation

Both producers stamp with `CLOCK_MONOTONIC` - `time.monotonic_ns()` in Python,
`clock_gettime(CLOCK_MONOTONIC)` in C++ - so streams are joined by nearest
timestamp with no clock fitting. Frames are the scarce resource, so the episode
is anchored on one camera and every other stream is matched to it.

Measured alignment: robot mean 0.29 ms / p95 0.50 ms. The wrist camera sits a
consistent ~7.7 ms from the external one; that is a fixed phase offset between
two free-running 30 fps streams, not jitter.

`--max-robot-skew-ms` is a **filter, not a warning**. The camera recorder
deliberately outlives the bridge, and a frame outside that overlap would
otherwise be paired with robot state seconds stale. Frames recorded while the
dead-man was released are dropped too - they are not demonstration.

### Validating an episode

```bash
.venv/bin/python validate_episode.py episodes/ep001
```

Checks the failure modes a summary line hides: missing or zero-byte frames,
frames reused because a camera lagged the anchor, action columns that never
vary, `q_robot` identical to `q_command` (which would mean measured state was
never recorded), a gripper that never actuated, and demonstration segments too
short to be useful. Run it before an episode joins a dataset.

First accepted episode (`ep001`): 704 frames at 30.02 Hz, 0 missing or reused
frames, all 7 joints active, gripper over its full range, one contiguous
segment, robot skew mean 0.242 ms.

### Action representation

`episode.csv` keeps `q_master` (raw lead arm), `q_command` (what was actually
sent after offset, scale, filtering and safety clipping) and `q_robot`
(measured) as separate columns. Roadmap section 13 is emphatic about this: the
training action is `q_command`, never the raw master encoder. The delta form
`a_t = q_command(t+1) - q_robot(t)` from section 14 is derived alongside it as
`dq_action*`.

## Tools

| Script | Purpose | Touches robot? |
|---|---|---|
| `check_ready.py` | Pre-flight gate check across lead arm, Franka, cameras | read-only |
| `monitor.py` | Live 8-servo read-out with per-joint travel ranges | read-only |
| `calibrate.py` | Guided joint-index / direction / range calibration | read-only |
| `bench_read.py` | Sustained SyncRead rate benchmark | read-only |
| `dump_state.py` | Full servo register inventory | read-only |
| `set_baud.py` | Change bus baud (writes EEPROM addr 8) | servos only |
| `tune_bus.py` | Change return delay (writes EEPROM addr 9) | servos only |
| `pnp7_lead.py` | Driver module used by the above | read-only |
| `verify_wrap.py` | Confirms signed decode near the encoder boundary | read-only |
| `make_teleop_config.py` | calibration.json -> pnp7_teleop.conf | none |
| `pnp7_teleop.cpp` | The teleop bridge itself | commands the arm |
| `mark_verified.py` | Records a confirmed joint direction | none |
| `analyze_run.py` | Validates a run against its configured limits | none |
| `record_cameras.py` | Dual RealSense recorder | none |
| `build_episode.py` | Joins teleop log + cameras into an episode | none |
| `collect_episode.sh` | Runs a whole episode end to end | commands the arm |
| `diag_gripper.py` | Measures real hand reaction latency from a log | none |
| `diag_jitter.py` | Command dither while holding still; `--lead-only` reads the encoders directly | read-only |
| `validate_episode.py` | Checks a collected episode is fit to train on | none |
| `check_correspondence.py` | Lead-vs-robot configuration drift | read-only |
| `rebase_calibration.py` | Move a verified calibration to a new posture | read-only |
| `prune_episode.py` | Reclaim space from a validated episode | none |
| `snapshot_state.py` | Capture the verified state as known-good | read-only |
| `view_cameras.py` | Show both RGB streams on the robot PC screen | none |
| `demo.sh` | Live teleop demonstration with cameras on screen | commands the arm |

## Usage

```bash
cd ~/workspace/pnp7_teleop
.venv/bin/python check_ready.py        # all gates must read PASS
.venv/bin/python calibrate.py --out calibration.json
```

## Franka pre-conditions

`check_ready.py` verifies the dead-man too -- that the device exists, opens,
and actually reports the configured key. It was added after a replug broke the
dead-man path while every other gate still read PASS. It reads
`deadman_device` and `deadman_key` from the teleop config rather than keeping
its own defaults, so the gate and the bridge cannot disagree about what they
are checking. A second row reports whether udev is keeping the desktop off the
button.

`check_ready.py` verifies each of these:

- STO released (`stoState: SafeTorqueOn`) - physical enabling device
- All 7 brakes released - unlock in Desk
- FCI port 1337 open
- Gripper port 1338 open - requires Franka Hand configured in Desk
- `robot_mode` 1 (Idle) or 2 (Move)

## Encoder decoding

Present Position is a signed int32 that the SDK returns unsigned. Two joints on
this arm rest on the boundary - J6 near tick 0, J5 near 4095 - so J6 backdriven
below zero came back as `2^32-3` and calibration reported a travel of
377487359 degrees.

Measured behaviour: the XL330 in Position Control Mode does **not** wrap
modularly in this range. It reports a continuous signed value that runs past the
0..4095 window (J6 observed at -397, J5 at 4104). So signed decoding is the
actual fix. The driver additionally accumulates a wrap-safe continuous tick
count, which is a no-op for the observed hardware but keeps a true wrap from
ever reaching the Franka as a ~360 degree step.

Use `ticks_cont` / `q_rad` for arithmetic; `ticks_raw` is kept for log fidelity.

## Safety design

Carried over from the roadmap and from the existing SpaceMouse controller:

- relative joint mapping with clutch, never absolute pose copying
- hold-to-enable dead-man; release freezes the target
- low-pass filter, then velocity limit, then acceleration limit
- lead-arm reads happen on their own thread, never inside the FCI callback
- start at `scale = 0.25`, raise only after directions are confirmed
- velocity and acceleration limits are PER JOINT (`max_joint_velocity` accepts
  one value or seven). A human rotates a wrist far faster than a shoulder, and
  the Franka's own dq limits differ across the arm, so a single global cap
  throttles the wrist while the big joints sit idle. Measured at scale 1.0 with
  a uniform 0.5 rad/s: J5 and J7 sat at the cap ~10% of held cycles while
  J1-J4 and J6 never reached it. `analyze_run.py` reports this as `vsat`.
- joint limits are applied to the desired position, before rate limiting
- velocity is additionally capped by the discrete-exact braking bound so a
  joint never approaches a target faster than it can stop
- the gripper runs on its own thread at ~40 Hz; `franka::Gripper::move` blocks
  and must never be called from the 1 kHz callback
- the gripper target is seeded from the MEASURED width at connect, and the
  thread stays disarmed until teleop first engages. Both matter: a target left
  at its default of 0.0 reads as "fully closed", and the hand slammed shut at
  launch before the operator had touched anything. `analyze_run.py` now reports
  any width motion occurring before the first dead-man press.

## Gripper latency

The Franka Hand is not a servo. `move()` blocks until the hand finishes
travelling, so a reversal mid-move was only honoured after the stale journey
completed. Measured on the first continuous build: median reaction latency
584 ms, worst 2010 ms, with the hand visibly closing all the way to 2 mm before
turning round to open.

Two facts govern any fix:

- `move()`/`stop()`/`grasp()` use TCP, `readOnce()` uses UDP, and they take
  different mutexes, so they do not block each other. `tcpBlockingReceiveResponse`
  releases `tcp_mutex_` on every poll iteration, so `stop()` genuinely can
  preempt an in-flight `move()`.
- `readOnce()` is a blocking UDP receive gated by the hand's state publishing,
  which is roughly 5 Hz and sometimes stalls for over a second.

The first preemption attempt failed because the preempt check shared a thread
with `readOnce()`, so it only evaluated a few times a second -- long after the
stale move had finished. The check does no I/O at all and now runs on its own
2 ms thread, with state reading isolated on a third.

**Production setting: binary mode, full 80 mm travel** (`full50b.conf`).
Continuous width tracking was measured and rejected -- it is structurally wrong
for fast reversals, because every intermediate width is a committed journey.

`--gripper-binary` sidesteps the problem: the hand opens or closes on a trigger
threshold with 0.08 hysteresis, so there are no intermediate moves to become
stale. Closing uses `grasp()` with a force target and a full-width epsilon,
because `move(0)` would stall against an object and report failure. The
continuous analogue trigger position is still recorded as
`gripper_master_ticks` at 1 kHz, so choosing binary for control does not
discard the analogue signal from the dataset.

Measured reaction latency: continuous 584 ms median, binary 128 ms median. Note
the hand publishes state every ~204 ms, so anything below that is at the
measurement floor rather than a real reading. Remaining latency is travel time
(80 mm at 0.1 m/s = 800 ms), which is physics, not software.
`--gripper-open-width` and `--gripper-speed` are the only levers on it; both are
left at full/default by choice.

## Command dither and the lead deadband

Symptom: motors audibly buzzing with the dead-man held while the operator was
holding still.

Cause, measured: the lead arm's encoder is quantised at 4096 counts/rev, and a
servo resting exactly on a count boundary flips between two adjacent values
indefinitely -- 134 changes/second on one measurement. At scale 1.0 a single
count is 1.534 mrad, so that becomes a tens-of-Hz command dither. The Franka's
position controller is stiff and chases it: motion too small to see, loud
enough to hear.

The 6 Hz low-pass cannot fix it. First-order rolloff attenuates ~30 Hz by only
about 20 dB, and the filter output changes every cycle regardless, so the
command is never truly constant.

Fix: a hysteresis (backlash) operator on the raw counts, applied before
scaling. The held value does not move until the input travels more than
`lead_deadband` counts, after which it tracks continuously offset by the band --
so chatter never propagates, while real motion has no dead zone and no steps.
Default 2 counts = 0.176 degrees of lead motion, well below hand tremor.

Measured A/B in `dry` mode, dead-man held, lead arm untouched:

| joint | deadband 0 | deadband 2 |
|---|---|---|
| J2 | 0.94 ct, 3 reversals/s | 0.00 ct, 0 |
| J4 | 1.00 ct, 11 reversals/s | 0.00 ct, 0 |
| J5 | 1.00 ct, 64 reversals/s, 0.512 mrad | 0.00 ct, 0 |

Which joint chatters depends on which one is parked on a count edge in that
pose, not on a faulty servo -- it was servo 1 in one measurement and J5 in
another. `diag_jitter.py --lead-only N` measures the encoder noise directly,
with no robot involved.

Some hum is inherent regardless: Franka joint-position control is stiff by
design and the motors are audible holding station against a perfectly constant
command. The deadband removes the dither, not the hum.

## Correspondence drift

Relative joint mapping never establishes absolute correspondence between the
two arms -- that is deliberate. The consequence is that moving the lead arm
while the dead-man is released changes the lead pose without moving the robot,
so the two can end up in different configurations.

When that happens on an upstream joint, every joint below it LOOKS mirrored
while its sign is perfectly correct. Rotating the lead forearm (J5) 180 degrees
inverts the wrist, so J6 appears to move backwards. Observed in practice: J5
drifted 206 degrees out and J6 read as reversed, despite J6 having been verified
in isolation and tracking at ratio 0.94 in the same run.

Continuous tick accumulation lives only for one process, so a neutral captured
after a joint wound past a revolution sits 4096 ticks from what a fresh process
reads at the same physical position. Correspondence is therefore compared
modulo one turn -- a real 180 degree offset still shows, a bookkeeping full turn
does not. Teleoperation was never affected by this, because it uses deltas from
a clutch origin captured live in the same process.

`check_correspondence.py` measures the drift against the calibration pose, and
`collect_episode.sh` runs it as a non-blocking pre-flight. Do not flip a
downstream sign until correspondence is restored -- a sign flipped to
compensate for a rotated upstream joint is only correct while that joint stays
rotated.

## Rebasing onto a new working posture

If the lead arm's comfortable configuration differs from the one calibration
was done in, rebase rather than recalibrate:

```bash
python rebase_calibration.py --set-sign J6=+1 --unverify J7 --label "J5+180"
```

The servo-to-joint mapping is physical wiring and does not depend on posture, so
it is never re-derived. `calibrate.py` is in fact unreliable in a gravity-
unstable posture: with torque off the wrist sags whenever an upstream joint
moves, and its travel can exceed the joint actually being swept. Observed in
practice -- servo 6 was picked for J1, J3 and J6 in one pass, which the
ambiguous/duplicate checks caught.

Only joints DOWNSTREAM of a rotated joint can change direction. Rotating the
forearm cannot alter how the base or shoulder move, so those signs carry over
with their verification intact.

Prefer `--set-sign Jn=+1` over `--flip`: flip is a toggle, and running a rebase
twice silently undoes it. The tool now refuses a repeat flip with the same
joints and label, and records every rebase in `posture_history`.

## Known-good state

```bash
python snapshot_state.py --label "ready-for-vla-collection"
```

Writes `known_good/` with the calibration, the working configs, and a
`state.json` recording the servo EEPROM settings, verified joint signs, camera
identities and accepted episodes. Re-running it later shows whether anything
has drifted -- it flags any servo whose baud, return delay or torque state is
not what was verified.

State captured 2026-08-20, posture `J5+180`:

| joint | servo | sign | |
|---|---|---|---|
| J1 | 1 | `+1` | verified |
| J2 | 2 | `-1` | verified |
| J3 | 3 | `+1` | verified |
| J4 | 4 | `-1` | verified |
| J5 | 5 | `+1` | verified |
| J6 | 6 | `+1` | verified, inverted vs the original posture |
| J7 | 7 | `+1` | verified |

Collection config: `full100b.conf` -- scale 1.0, session clamp 1.0 rad,
per-joint velocity `0.45 x4 / 0.6 / 0.55 / 0.6`, binary gripper at full travel.

## Live demonstration

```bash
./demo.sh                 # 5 minutes
DURATION=600 ./demo.sh    # 10 minutes
```

Runs the pre-flight and correspondence checks, opens both RGB streams on the
robot PC's screen, then starts teleoperation. Under the camera panes a status
strip shows the state machine, whether the dead-man is held, all seven measured
joint angles, and the gripper opening -- live. Ctrl-C ends it and prints the
run analysis.

The bridge publishes that state via `status_path` in the config. Only atomic
stores happen in the 1 kHz callback; a separate thread serialises JSON at 10 Hz
and installs it with an atomic rename, so a reader never sees a partial file
and the realtime loop is untouched.

## Inspecting the cameras

```bash
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  .venv/bin/python view_cameras.py
```

Must run on the machine's own display. Opens the same devices, resolution and
format the recorder uses, so what you see is what lands in an episode. `q`
quits, `s` saves stills. Qt font warnings on stderr are cosmetic.

## Commissioning record

All seven joint directions were verified live under supervision at scale 0.25,
one joint at a time with the rest masked, before any scale increase:

| joint | servo | sign | note |
|---|---|---|---|
| J1 | 1 | `+1` | calibration read `-1`; corrected after live test |
| J2 | 2 | `-1` | |
| J3 | 3 | `+1` | |
| J4 | 4 | `-1` | |
| J5 | 5 | `+1` | crosses the encoder boundary in normal use |
| J6 | 6 | `-1` | rest position sits near tick 0 |
| J7 | 7 | `+1` | first joint driven, roadmap Step C |

Measured tracking at scale 0.25: RMS error 0.0002-0.0006 rad per joint, peak
under 0.0015 rad, lag ~7 ms, at 1000 Hz with zero lead-arm read failures.

Masked joints were confirmed frozen to exactly `0.0000` command range across
full sessions, so single-joint tests really were single-joint.

Scale 1.0 shakeout: tracking RMS 0.00011-0.00098 rad, lag 6-9 ms, unchanged
from scale 0.5 -- the arm keeps up at 1:1. Velocity saturation on J5/J7 led to
per-joint limits (`full100b.conf`).

Scale 0.5 with all seven joints and the hand enabled: tracking RMS
0.00007-0.00089 rad, lag 7-13 ms, largest command step exactly at the velocity
limit, 30 gripper commands with 0 errors. Trigger maps 649 ticks (squeezed,
closed) to 1355 (released, open); confirmed by the operator.
