# GELLO (PNP-7) -> Franka teleoperation

Implements V1 of `pnp7_roadmap.md`: relative joint-space teleoperation from the
GELLO (PNP-7) to the Franka, for VLA/imitation-learning data collection.

## Hardware as discovered

Nothing about this bus was documented, so it was identified by probing.

| | |
|---|---|
| GELLO bus | 8x Dynamixel XL330, Protocol 2.0, half-duplex behind an FT232H |
| GELLO device | `/dev/gello` (udev symlink, stable across replug) |
| Joints | IDs 1-7 = J1..J7, XL330-M288-T (model 1200) |
| Gripper trigger | ID 8, XL330-M077-T (model 1190) |
| Firmware | 52 on all servos |
| Franka | FR3 @ `172.16.0.2` via `enp4s0` (172.16.0.1/24) -- see "libfranka version" below |
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
python diag/tune_bus.py --raw 250 --yes
python diag/set_baud.py --from-baud 1000000 --to 57600 --yes
```

A udev rule (`/etc/udev/rules.d/99-pnp7-lead.rules`) pins the adapter to
`/dev/gello` and sets the ftdi_sio `latency_timer` to 1 ms (the 16 ms
default alone cost ~16 ms per cycle). `franka` was added to the `dialout` group.

The same rule pins the foot brake to `/dev/foot_brake`.
Legacy `/dev/pnp7_lead` and `/dev/pnp7_deadman` aliases remain available
for older configurations. The foot brake takes priority over a connected
SpaceMouse for the legacy deadman alias. This was learned
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
  `/dev/foot_brake_cfg`; interfaces 01 and 02 stay root-only, because those
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

## libfranka version

**2026-09: the arm's firmware was upgraded past FR3 System Version 5.9.0, and
libfranka 0.15.0 stopped connecting.** Rebuilt at **0.21.3**, which is what
`bin/pnp7_teleop` now links against.

libfranka pins a minimum firmware per release: 0.15.0 declares `>= 5.7.2`,
0.18.0 declares `>= 5.9.0`. Once the robot crossed 5.9.0 everything below 0.18.0
was out. Of what remained, **0.20.0 is the floor worth taking**: 0.18.0 changed
`RobotState` to float-based fields, which breaks every place the bridge assigns
`robot_state.q` (and `dq`, `tau_J`, `O_T_EE`, `O_F_ext_hat_K`) straight into a
`std::array<double, 7>` -- about thirteen of them. 0.20.0 reverted that, so on
0.20+ they compile untouched.

Nothing else the bridge uses changed across 0.15 -> 0.21: `Robot::control`,
`JointPositions`, `RobotMode`, `RobotState::current_errors`, `Gripper` and
`Duration` have no changelog entries. The one 0.18 deprecation that could have
applied -- `computeUpperLimitsJointVelocity` / `computeLowerLimitsJointVelocity`
-- came in through `<franka/rate_limiting.h>`, which was included but never
referenced; that include has been dropped.

### Rebuilding it

Built **beside** the 0.15.0 tree rather than over it, so nothing else that links
against 0.15 changes and reverting is a matter of not setting `LIBFRANKA`:

```bash
git clone --recursive --branch 0.21.3 --depth 1 \
    https://github.com/frankarobotics/libfranka.git ~/catkin_franka/libfranka-0.21.3
cd ~/catkin_franka/libfranka-0.21.3
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    -DCMAKE_PREFIX_PATH=/opt/openrobots        # <- pinocchio lives here
cmake --build build -j"$(nproc)"

cd ~/workspace/andyls/pnp7-lead-teleop
LIBFRANKA=$HOME/catkin_franka/libfranka-0.21.3 ./build.sh
```

`-DCMAKE_PREFIX_PATH=/opt/openrobots` is the whole trick. **libfranka 0.20+ adds
a hard, unguarded `find_package(pinocchio REQUIRED)`** -- there is no option to
turn it off, even though this bridge never touches `franka::Model`. pinocchio
3.4.0 is already installed on the robot PC from robotpkg, but it lives under
`/opt/openrobots`, which CMake does not search by default, so the configure step
fails with a bare "could not find pinocchio" that reads like a missing package.
Nothing needs installing; it only needs pointing at.

The rest of the toolchain clears the bar without help: 0.21.3 asks for CMake
>= 3.16 and C++17, and the robot PC has 3.16.3 and g++ 9.4.0 on Ubuntu 20.04.
Eigen 3.3.7 and Poco 1.9.2 are the system ones; `fmt` is fetched at configure
time, so the machine needs network access for that step.

### Why not conda-forge / pixi *on this machine*

`libfranka` is on conda-forge, up to 0.21.3, and installing a prebuilt one is a
reasonable instinct -- it just does not fit this machine. The conda build
targets a different dependency stack from this one's:

| | conda-forge 0.21.3 | this robot PC |
|---|---|---|
| libstdc++ | `>= 14` | g++ 9.4, GLIBCXX_3.4.28 |
| Eigen | `eigen-abi >= 5.0.1` | 3.3.7 |
| Poco | `>= 1.15.3` | 1.9.2 |
| pinocchio | `>= 4.1.0` | 3.4.0 |

Taking it would mean compiling the bridge inside the environment with
conda's `gxx` against conda's Eigen and Poco, and rebuilding `DynamixelSDK`
there too -- a toolchain migration under a 1 kHz realtime control loop whose
timing is measured and recorded in this file, in exchange for skipping a
two-minute compile. The source build links against exactly the same system
Eigen and Poco that `build.sh` compiles the bridge against, which is the
property worth keeping.

Every line of that argument is about this machine's *pre-existing state*, and
none of it survives on a bare box -- which is why `robot-s0`, commissioned from
nothing in 2026-09, does the exact opposite. See "Setting up robot-s0" below.

(pixi would earn its place on the *Python* side instead -- specifically the
LeRobot export, which needs `lerobot` and `torch` from the RLinf environment and
currently has no reproducible definition at all. See `gui/README.md`.)

### Still to check on the robot

- **The hardcoded joint limits.** `kQMin` / `kQMax` in `src/pnp7_teleop.cpp` are
  commented "Franka Panda joint limits" and carry Panda's numbers. FR3's
  envelope differs -- most visibly on J6, where Panda allows roughly -1 deg and
  FR3 starts near +25 deg -- and FR3's envelope was itself widened at system
  5.9.0. So the compiled ceiling can no longer be assumed to be the more
  conservative of the two. It has not bitten because the calibration posture
  keeps J6 near 173 deg, but it should be read off the installed firmware rather
  than off a datasheet.

## Setting up a checkout

Three of the things this repo needs are deliberately not tracked: `bin/` and
`.venv/` are build products, and `DynamixelSDK/` is a 43 MB vendored upstream
tree. A fresh checkout is therefore source only, and `demo.sh` fails on its
first line with no `.venv/bin/python`. This is what happened when the working
copy moved to `~/workspace/andyls/pnp7-lead-teleop`; the recovery is below.

Already present on the robot PC:

| | |
|---|---|
| libfranka | 0.21.3 at `~/catkin_franka/libfranka-0.21.3` -- see "libfranka version" above. The old 0.15.0 tree is still at `~/catkin_franka/libfranka` and is no longer usable |
| pinocchio | 3.4.0 under `/opt/openrobots` (robotpkg); libfranka 0.20+ requires it |
| Eigen | `/usr/include/eigen3` (3.3.7) |
| `uv` | `/usr/bin/uv` |

```bash
cd ~/workspace/andyls/pnp7-lead-teleop

# 1. Vendored Dynamixel SDK -- the C++ tree the bridge links against.
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git -C DynamixelSDK checkout 2ded684              # what this was commissioned on
make -C DynamixelSDK/c++/build/linux64            # -> libdxl_x64_cpp.so

# 2. Python environment for the tooling. The editable install is what makes
#    `from pnp7.lead import ...` work from scripts in subdirectories.
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -r requirements.txt
uv pip install --python .venv/bin/python -e .

# 3. The bridge. LIBFRANKA is required now that the default path holds the
#    unusable 0.15.0 build -- see "libfranka version" above.
LIBFRANKA=$HOME/catkin_franka/libfranka-0.21.3 ./build.sh
```

`build.sh` takes `LIBFRANKA` and `DXL` from the environment, so a machine that
keeps either tree somewhere else does not need the script edited.

Then confirm the checkout before trusting it, in this order -- each step needs
strictly more hardware than the one above it, so the first failure tells you
which layer is wrong:

```bash
./bin/pnp7_teleop selftest conf/demo.conf     # offline; expect SELFTEST_OK, 11 checks
.venv/bin/python check_ready.py --config conf/demo.conf
./bin/pnp7_teleop dry conf/demo.conf 10 /tmp/dry.csv   # expect ~940 Hz, 0 read failures
```

`check_ready.py` needs the venv's interpreter specifically, and the failure mode
if you forget is quiet rather than loud. The system `python3` happens to carry
`cv2`, `numpy` and `pyrealsense2` but not `serial` or `dynamixel_sdk`, so it
does not crash -- it prints one `[FAIL] lead arm driver import -- No module
named 'serial'` row and then simply omits the torque and sample-rate gates. The
result is an `overall: FAIL` that reads like an ordinary hardware fault while
two of the four lead-arm checks never ran at all.

### Setting up robot-s0

`robot-s0` (`ssh robot-s0`, `liushuai@192.168.1.104`) is the second controller
PC, brought up from bare metal in 2026-09. It is set up the opposite way from
the robot PC above: everything below the application code comes from
conda-forge through `pixi`, and `pixi.lock` is the first reproducible
definition the C++ side of this project has ever had.

The argument in "Why not conda-forge / pixi" inverts point by point, because
all of it was about the robot PC's pre-existing state:

| The robot PC's reason | Why it does not hold on robot-s0 |
|---|---|
| pinocchio is already installed | It is not. From scratch that is a robotpkg apt repo plus root; conda-forge hands it over as a dependency of `libfranka` for free |
| ABI consistency with the existing stack | There is no existing stack. The whole thing is chosen at once, which makes it *more* internally consistent than the robot PC, not less |
| Do not migrate the toolchain under a measured 1 kHz loop | Nothing here is commissioned or characterised yet, so there is no measurement to disturb |
| ...in exchange for skipping a two-minute compile | It is only two minutes because pinocchio was already there. From scratch it is apt repo setup, Poco/Eigen/cmake dev packages, then libfranka, then DynamixelSDK -- and most of it needs root, which this account does not have |

The condition attached is **take the whole environment, not half of it**.
conda-forge's libfranka 0.21.3 wants libstdc++ >= 14, Eigen 5 and Poco 1.15, so
the bridge is compiled with the env's `gxx` *and DynamixelSDK is built there
too*, since the bridge links `libdxl_x64_cpp.so`. A DXL built by the system
compiler would reintroduce exactly the split the source build on the robot PC
exists to avoid.

That last point has a sharp edge worth naming: the SDK's Makefile sets `CC`,
`CX` (its spelling for the C++ compiler) and, separately, `LD = g++` for the
shared-library link. `pixi.toml`'s `dxl-build` task overrides all three.
Overriding only the compilers compiles the objects in-env and then links the
final `.so` with `/usr/bin/g++`, which looks like it worked.

(In practice the bridge would probably survive mixing -- it only touches
`Robot`, `RobotState`, `JointPositions`, `Duration` and `Gripper`, all plain
`std::array`/POD across the API, with Eigen appearing only in `franka::Model`,
which it never uses. But "the Eigen types happen not to cross the ABI boundary"
is not a bet worth taking on a robot safety chain when building in-env is free.)

From a bare checkout:

```bash
curl -fsSL https://pixi.sh/install.sh | bash     # -> ~/.pixi/bin, edits .bashrc
cd ~/workspace/pnp7-lead-teleop
pixi install         # solve + materialise, ~1.7 GB under .pixi/
pixi run setup       # DynamixelSDK at 2ded684, built in-env, then the bridge
pixi run selftest    # offline; expect SELFTEST_OK, 11 checks
```

`build.sh` needs no arguments there. An explicit `LIBFRANKA` still wins, so the
robot PC's documented invocation is untouched; with none set it falls back to
`$CONDA_PREFIX/include` + `$CONDA_PREFIX/lib` when it finds libfranka headers
there, which is what `pixi run` supplies. `.pixi/` is ignored; `pixi.lock` is
tracked, because it is the half that reproduces.

What the environment pins:

| | |
|---|---|
| libfranka | 0.21.3 from conda-forge, pulling libpinocchio 4.1.0, Poco 1.15.3, fmt 12.1 and the Eigen 5.0.1 ABI |
| Compiler | conda-forge gcc 14.4.0 -- the floor `libstdcxx >= 14` actually asks for rather than the newest available, because the vendored DynamixelSDK is a 2023 tree with no reason to survive gcc 16's stricter defaults |
| Python | 3.11, with the exact pins from `requirements.txt` and `pnp7` installed editable |

`ldd bin/pnp7_teleop` resolves libfranka, libstdc++, libgcc_s, Poco, fmt and
pinocchio inside `.pixi/envs/default`; only glibc's own libraries come from the
system, which is as far as an unprivileged env can go.

#### What is not commissioned on robot-s0 yet

The environment imports and offline selftest pass. `check_ready.py` checks
the connected devices, but dependent checks are skipped after a prerequisite
fails; it does not validate the RT kernel or scheduling permissions.

2026-09-05 follow-up: `liushuai` now has sudo access. GELLO and the foot brake
are connected and the udev rules are installed. The canonical names are
`/dev/gello` (currently `ttyUSB0`) and `/dev/foot_brake` (currently `event24`,
`KEY_F3`). The account belongs to `dialout`, `input`, and `plugdev`; existing
sessions were also granted ACL access to these device nodes. New sessions
inherit the groups. The FTDI latency timer is 1 ms.

All eight GELLO servos were found at 57600 baud and changed to 1000000 baud
using `diag/set_baud.py`; every baud register was read back successfully.
A one-second readiness sample measured 495 Hz with zero failed frames and
all eight servos passive. A physical foot-brake test then confirmed
`KEY_F3` held continuously and released after 15.1 seconds of observation.

Camera access was granted using `udev/99-robot-s0-cameras.rules`, scoped to
D435i serials `216623060304` and `229123060429`. Named `liushuai` ACLs were
applied to the two USB nodes, twelve video nodes and two hidraw nodes;
read/write permission was verified as that user. Existing ownership and
other users' access were preserved. Rules were reloaded for future events,
without retriggering or resetting connected devices. No process held the
camera nodes at inspection time. Per the operator's request, no camera SDK
connection or streaming test was performed; enumeration and image capture
remain unverified after this permission change.

For the requested temporary non-RT trial, both `robot` and `home` explicitly
use `RealtimeConfig::kIgnore`. This bypasses the startup checks below; it does
not provide realtime scheduling or establish 1 kHz performance. The old
`pnp7` controller has not been changed.

| Gap | Effect | Fix |
|---|---|---|
| `ulimit -r` is 0 for `liushuai`. `/etc/security/limits.d/99-franka-raojiaji.conf` grants rtprio 99 to `raojiaji` only | libfranka's default `RealtimeConfig::kEnforce` refuses to start `robot` mode | add a `liushuai - rtprio 99` limits.d entry, then log out and back in |
| Kernel is `6.8.0-138-generic`, PREEMPT_DYNAMIC -- not PREEMPT_RT. The robot PC runs `5.15.197-rt91` | the 1 kHz FCI loop has no realtime guarantee, so none of the timing recorded in this file carries over | install a PREEMPT_RT kernel before characterising anything |
| Camera SDK enumeration and capture have not been retested after the ACL fix | device permissions pass, but images are not yet validated | test when the operator is ready and the cameras are free |
| FCI port 1337 refuses while ICMP to `172.16.0.2` succeeds | the arm is wired and reachable on `enp131s0` (172.16.0.1/24) but FCI is not active | enable FCI in Desk and release the brakes |

One more thing to know about the host alias: `robot-s0` points at
`192.168.1.104`, which is the *WiFi* interface (`wlx60a3e345f593`). The machine
also has `192.168.0.4` on USB ethernet and `172.16.0.1` on the FCI NIC. If the
WiFi lease changes, the SSH alias must be updated to the new IP -- unlike `pnp7`, which is pinned to a
Tailscale address.

### Syncing the checkouts

There are three working copies, all tracking
`github.com/handysome6/pnp7-lead-teleop`:

| | |
|---|---|
| laptop | `~/workspace/vla_data_collect/pnp7-lead-teleop` |
| robot PC | `~/workspace/andyls/pnp7-lead-teleop` |
| robot-s0 | `~/workspace/pnp7-lead-teleop` |

They reach the remote over different protocols, for reasons that are not
obvious from `git remote -v`:

- **Laptop: HTTPS.** Tailscale resolves `github.com` to `198.18.0.47` and closes
  port 22, so the `git@github.com:` form cannot connect. HTTPS works, using the
  `gh` credential helper. If `github.com` is ever taken back out of Tailscale's
  interception, the remote can be switched to SSH.
- **Robot PC: SSH through a `github-pnp7` host alias.** This machine's default
  keys belong to a different GitHub account (`git-xuxin`) which has no write
  access here, and GitHub decides the identity from whichever key authenticates
  first. The alias in `~/.ssh/config` pins a write-enabled deploy key with
  `IdentitiesOnly yes`; that option is load-bearing, not decoration. Drop it and
  ssh offers the default key first, authenticates as `git-xuxin`, and the push
  is refused.
- **robot-s0: HTTPS.** Nothing intercepts `github.com` there and the repo is
  public, so a plain clone works with no key setup at all. A push will need
  credentials; it has none yet.

The robot PC also carries a repo-local `user.name` / `user.email`, because its
global git identity belongs to another user of the same account. Without it,
commits made at the robot are attributed to that person. robot-s0 carries the
same override for a nearer version of the same reason -- the `liushuai` account
is shared, and its global git identity is simply unset, so commits made there
would otherwise be attributed to `liushuai@robot-s0`.

Check which identity a push will use with `ssh -T git@github-pnp7` -- it answers
with the repo name for a deploy key, or a username for an account key.

## Teleoperation bridge

`src/pnp7_teleop.cpp` implements roadmap V1. Two threads, per roadmap section 12:
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
3. notch on the mapped delta (`notch_hz`, `notch_q`; `notch_hz=0` disables),
   re-clamped to the session bound afterwards because a notch overshoots a step
   by ~16% at Q 2.0. See "Follower mode at ~5 Hz" below
4. **joint-limit clamp on the desired position** - before rate limiting, never
   after; clamping the output instead lets the clamp emit a step of arbitrary
   size that bypasses the velocity and acceleration limits
5. low-pass filter (`lowpass_hz`), with the filter state clamped so it cannot
   wind up outside the envelope
6. velocity limit, capped additionally by the discrete-exact braking bound
   `sqrt(2*a*d + (a*dt)^2) - a*dt`, so a joint never enters the target faster
   than it can stop
7. acceleration limit

Releasing the dead-man, a stale lead arm, or `SIGINT` all route to `hold()`,
which decays velocity to zero and freezes. A session only reports
`motion_finished` once every joint is actually at rest, because libfranka
rejects a motion that ends with non-zero velocity.

Config values are validated against compiled ceilings and can only ever be more
conservative than them.

Every run prints the shaping actually in force -- `command shaping:
lowpass_hz=6 deadband=2 counts notch=4.8Hz Q=2` -- in `robot` and `dry` mode
alike, so a recorded episode's `bridge.log` says what produced it. The CSV
cannot answer that question after the fact.

#### Follower mode at ~5 Hz

Measured 2026-09-12 over four deliberate grasp approaches
(`artifacts/vibration_audit_20260912`): descending toward the cube, the arm
develops an oscillation the command does not contain. `q_robot - q_target`
reaches 0.068-0.098 deg at 4.44-5.22 Hz carrying 9-18 Nm of same-band J2
torque, and it grows about twentyfold as the arm reaches down, peaking at the
lowest point. There is no contact -- the operator confirmed it and `O_F_ext`
agrees, with Fz less negative at the bottom than high up. What keeps exciting
the mode is single-count stepping of the lead arm: one count is 1.5 mrad, which
at this reach is ~1.1 mm of fingertip travel, and a step carries energy
everywhere.

`lowpass_hz` is the wrong instrument. Being first order, reaching 5 Hz means
dragging the 1-2 Hz band the operator works in down too: at 3 Hz it cuts 5 Hz
only to 0.514 while adding 24.9 ms of lag at 1 Hz. The notch at 4.8 Hz, Q 2.0
cuts 4.4-5.2 Hz to 0.31-0.33 for 17.3 ms, and leaves 2 Hz at 0.970. Replaying
the recorded lead signal through it drops the 4-6 Hz band to 0.42-0.68 of its
original amplitude while the 1-2.5 Hz band lands within 3% of unchanged.

Retune `notch_hz` if the mode moves. Its frequency is the Franka's fixed joint
impedance over the arm's effective inertia, so a different tool or a distinctly
different working posture will shift it.

Two things this does not fix. The tremble the operator sees at the bottom is
about 9 mm peak-to-peak over 1.5-6 Hz, and only ~1.7 mm of that is the arm --
the remaining ~8 mm below 2.5 Hz is hand motion, where `q_target` exceeds the
arm-generated part by 10-50x and shares a band with the operator's intent, so
no filter reaches it. And the tool is still declared as a Franka Hand except
for its mass; see the Robotiq notes.

### Build and run

On robot-s0, build with `pixi run bridge`. The compiled binary runs without
`pixi run` in front: `build.sh`
bakes an RPATH onto `.pixi/envs/default/lib`, so the binary finds libfranka and
libstdc++ from a bare shell. The RPATH is absolute, though, which is the catch
-- move or delete the env, or relocate the checkout, and the binary stops
loading until `pixi run bridge` relinks it. The *Python* entry points
(`check_ready.py`, everything under `collect/` and `calib/`) do need `pixi run`
or `pixi shell`.

```bash
pixi run bridge  # robot-s0; use ./build.sh with LIBFRANKA on the old robot PC
./bin/pnp7_teleop selftest conf/full50b.conf             # offline, 11 checks
./bin/pnp7_teleop home     conf/full50b.conf             # drive to home_qpos
./bin/pnp7_teleop dry      conf/full50b.conf 30 dry.csv   # hardware, no robot
./bin/pnp7_teleop robot    conf/full50b.conf 60 run.csv   # live
```

`dry` runs the entire pipeline - lead arm, dead-man, clutch, safety chain -
against a simulated rest pose, so mapping and directions can be confirmed with
the robot untouched. Always run it before `robot`.

## Data collection

```bash
DURATION=60 CONF=conf/full50b.conf scripts/collect_episode.sh episodes/ep001
```

Starts both cameras, waits for `CAMERAS_READY` (auto-exposure needs to settle
before frames are worth training on), runs the bridge, then joins everything.
`MODE=dry` exercises the whole path without commanding the robot.

`gui/` runs the same sequence from a browser instead, with the operator deciding
where each take starts and ends rather than passing a duration up front. It also
owns the cleanup the shell script only prints: `set -euo pipefail` makes
`collect_episode.sh`'s `BRIDGE_RC` unreachable, and its early exits leave the
camera recorder orphaned holding both RealSense pipelines. See `gui/README.md`.

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
.venv/bin/python collect/validate_episode.py episodes/ep001
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

## Layout

Everything used to sit in the root directory, which made it impossible to tell
the three scripts you run every day from the twenty that were written once to
identify a servo bus. Grouped by what a file is *for*:

```
build.sh  demo.sh  check_ready.py        the three entry points
pixi.toml  pixi.lock                     robot-s0's whole environment, C++ included
calibration.json                         live calibration, read by most tools
conf/                                    12 teleop configs
src/pnp7_teleop.cpp                      the realtime bridge
pnp7/lead.py                             lead-arm driver (the only shared module)
scripts/                                 collect_episode.sh, collect_batch.sh
calib/                                   calibration, config generation, drift checks
collect/                                 recording, episode building, validation,
                                         viewer, LeRobot export
gui/                                     browser operator panel over the above
diag/                                    diagnostics, benchmarks, bus tuning, bring-up probes
deadman/                                 button identification and the udev rule
event_camera/                            EVK4 tooling (separate subsystem)
known_good/                              restorable snapshot, written by calib/snapshot_state.py
archive/                                 superseded configs and one-off patch scripts
```

`pnp7/` is a real package installed editable -- into `.venv` on the robot PC,
into the pixi env on robot-s0 -- so `from pnp7.lead import ...` resolves no
matter which subdirectory a script lives in. Python puts
the *script's* directory on `sys.path`, never the repo root, so without the
install the eight importers would each need a `sys.path` shim.

Most scripts resolve `calibration.json` and `conf/*.conf` relative to the
current directory, so **run them from the repo root**, as every example here
does.

One exception worth knowing about: `event_camera/` is the source of truth, but
not what actually runs. The EVK4 needs OpenEB 5.3, so `collect_episode.sh`
launches it inside a container that mounts `~/metavision` as `/work` and runs
`record_events.py` from there. That directory is a manual copy. It is currently
byte-identical to the repo, but nothing enforces that -- edit a file under
`event_camera/` and the collection pipeline keeps running the old one until the
copy is refreshed.

## Tools

| Script | Purpose | Touches robot? |
|---|---|---|
| `check_ready.py` | Pre-flight gate check across lead arm, Franka, cameras | read-only |
| `demo.sh` | Live demonstration, cameras plus teleop | commands the arm |
| `scripts/collect_episode.sh` | Runs a whole episode end to end | commands the arm |
| `scripts/collect_batch.sh` | Several episodes in one sitting, validating each | commands the arm |
| `src/pnp7_teleop.cpp` | The teleop bridge itself | commands the arm |
| `pnp7/lead.py` | Driver module used by everything below | read-only |
| `calib/calibrate.py` | Guided joint-index / direction / range calibration | read-only |
| `calib/make_teleop_config.py` | calibration.json -> conf/pnp7_teleop.conf | none |
| `calib/check_correspondence.py` | Lead-vs-robot configuration drift | read-only |
| `calib/rebase_calibration.py` | Move a verified calibration to a new posture | read-only |
| `calib/mark_verified.py` | Records a confirmed joint direction | none |
| `calib/verify_wrap.py` | Confirms signed decode near the encoder boundary | read-only |
| `calib/snapshot_state.py` | Capture the verified state as known-good | read-only |
| `collect/record_cameras.py` | Dual RealSense recorder | none |
| `collect/build_episode.py` | Joins teleop log + cameras into an episode | none |
| `collect/validate_episode.py` | Checks a collected episode is fit to train on | none |
| `collect/prune_episode.py` | Reclaim space from a validated episode | none |
| `collect/view_cameras.py` | Both RealSense streams on the robot screen | none |
| `collect/export_lerobot.py` | Built episodes -> LeRobot, offline; needs the RLinf env | none |
| `gui/app.py` | Operator panel: previews, episode control, arm restore | commands the arm |
| `diag/analyze_run.py` | Validates a run against its configured limits | none |
| `diag/monitor.py` | Live 8-servo read-out with per-joint travel ranges | read-only |
| `diag/diag_gripper.py` | Measures real hand reaction latency from a log | none |
| `diag/diag_jitter.py` | Command dither while holding still; `--lead-only` reads the encoders directly | read-only |
| `diag/diag_pose.py` | Lead vs Franka joint configurations across a run | none |
| `diag/dump_state.py` | Full servo register inventory | read-only |
| `diag/bench_read.py` | Sustained SyncRead rate benchmark | read-only |
| `diag/set_baud.py` | Change bus baud (writes EEPROM addr 8) | servos only |
| `diag/tune_bus.py` | Change return delay (writes EEPROM addr 9) | servos only |
| `diag/probe_ping.py`, `diag/probe_listen.py` | Bring-up probes that identified the bus | read-only |
| `deadman/find_button.py` | Finds an unknown button's event node and key code | none |
| `deadman/grab_button.py` | Takes the button exclusively, standalone | none |
## Usage

```bash
cd ~/workspace/andyls/pnp7-lead-teleop
.venv/bin/python check_ready.py        # all gates must read PASS
.venv/bin/python calib/calibrate.py --out calibration.json
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
python calib/rebase_calibration.py --set-sign J6=+1 --unverify J7 --label "J5+180"
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
python calib/snapshot_state.py --label "ready-for-vla-collection"
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
  .venv/bin/python collect/view_cameras.py
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
