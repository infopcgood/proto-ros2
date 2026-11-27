# Continuous Prootface ROS pipeline

This package consumes the existing `osf_data`, applies the Prootface wearer
profile and causal controller, and renders a synchronized two-view BGR atlas.
It uses the existing screen-control service and dispatcher. All Prootface input,
preview and atlas-output adapters live inside this new package. Existing packages
retain their pre-integration code and original entry points.

[ROS API reference](API.md) lists topics, payload fields, read-only startup
parameters, launch arguments, ownership and the LED mapping API.

## Install

Use the same Python ABI/environment as rclpy. For WS hardware, that environment
must also provide the installed smileds driver. A venv needs system site packages
(or equivalent access to the ROS Python packages).

On a Mesa-based Debian/Ubuntu system:

```sh
sudo apt install libegl1 libgles2 libgl1-mesa-dri python3-numpy python3-pil python3-opencv python3-venv
```

Install the separate public core checkout; do not copy private assets into either
repository or into package data:

```sh
python3 -m venv --system-site-packages ~/prootface-venv
source ~/prootface-venv/bin/activate
python -m pip install -e "$HOME/prootface[render]"
```

Source the ROS Jazzy installation appropriate to the machine, then build:

```sh
cd ~/proto-ros2
python -c 'from colcon_core.command import main; raise SystemExit(main())' build --symlink-install --packages-select interface_pkg screen_dispatcher_pkg prootface_pkg
source install/setup.bash
```

Invoking colcon through the active Python is deliberate: generated node entry
points must use the environment containing the core and optional driver. A
system-installed `colcon` executable can otherwise retain a system-Python
shebang even while a venv is activated.

The core dependency is pinned in the integration workflow. EGL can select a
software Mesa renderer on a host without a GPU; the node logs the actual driver.
Check that log before drawing conclusions about on-device performance.

## Start with a desktop preview

```sh
ros2 launch prootface_pkg face.launch.py \
  character:=/path/to/private/character.json \
  profile:=/path/to/private/calibration.json \
  backend:=opencv
```

The character requires a matching PNG texture sidecar. OpenSeeFace sends to
127.0.0.1:11573 by default. This launch starts the optional Prootface-owned UDP input adapter; do not also
start the standalone Prootface UDP server. If your existing receiver already runs, pass
`start_receiver:=false`. The legacy PNG face node must not run and reclaim
screen ownership during this launch.

The calibration UI is http://127.0.0.1:8080. Use its five capture stages and save
the wearer profile. UI commands and ROS input share one Application state.
`web_port:=-1` disables HTTP. A remote operator can tunnel the loopback port;
changing `web_host` is an explicit deployment choice.

The controller stays at 24 Hz. Use `presentation_fps:=12` or `6` to hold display
poses longer. Default virtual per-side dimensions are 256x144; these are render
canvas dimensions, not a statement about physical LED count or panel geometry.
The atlas is twice the side width. Use `side_width` and `side_height` arguments
to change it. `backend:=none` runs the full publisher without a window or driver.

Other launch arguments: `style` (optional JSON of supported style parameters),
`udp_host`, `udp_port`, `egl_platform` (`auto`, `surfaceless`, `default`), `web_host`
and `web_port`. Use `face_id` to select a tracked face (default zero), and
`input_topic` to select another OSFData topic if the receiver is remapped.

## External expression buttons

The face node subscribes to facial_expression using the existing FacialExpression
message. Buttons select a latched expression; OpenSeeFace continues driving
speech. No emotion inference is performed.

```sh
ros2 topic pub --once /facial_expression interface_pkg/msg/FacialExpression \
  '{type: emotion, expression: angry, override_time: 0.0}'
```

Available names depend on the compiled character: neutral, happy, angry, sad and
unamused. type=emotion (or face) selects both eye/mouth banks; type=eyes or mouth
selects one independently. Zero override_time latches until another command or
restart. A positive duration temporarily overrides the latched choice and then
returns. Repeated identical commands do not extend the hold. idle and smile are
aliases for neutral and happy; other legacy PNG/phoneme names are not guessed.

Launch arguments expression_topic (default facial_expression) and
expression_transition (default 0.2 seconds, range 0–2) configure this new node.
Transitions run at 24 Hz while speech continues, with contact and full blinks
retaining priority. Tracking loss closes the mouth while preserving the chosen
expression. Older compiled characters support neutral only and reject missing
banks; install a newly compiled private character pair for the added poses.
HTTP buttons can alternatively POST {"expression":"angry"} to /api/expression.

## Dense WS hardware

```sh
ros2 launch prootface_pkg face.launch.py \
  character:=/path/to/private/character.json \
  backend:=ws panel_map:=/path/to/measured-panel-map.json \
  side_width:=256 side_height:=144 driver_strips:=8
```

Provide the actual measured map. No physical map is bundled. Its JSON schema:

| Key | Meaning |
| --- | --- |
| schema | 1 |
| side_width / side_height | Virtual pixels in one anatomical view |
| leds_per_strip | Slots per strip, matching the driver initialization |
| num_strips | Must match `driver_strips` and the installed driver build |
| pixels | Exactly leds_per_strip × num_strips entries in physical output order |

Each entry is `["left", x, y]`, `["right", x, y]`, or `null` for an unused/black
slot. Coordinates are integer pixels within one side, with top-left origin.
Entries run strip-major then LED index. Encode physical rotation, wiring order
and orientation in the map. Do not mirror a side again: the renderer has already
produced its anatomical view. BGR-to-RGB conversion happens once in this backend.

The current smileds API initializes LEDs-per-strip and brightness but does not
expose a dynamic strip-count argument. `driver_strips` describes the installed
driver build; it does not reconfigure that build. A mismatch is rejected.
Brightness remains a driver-specific integer; the existing default is retained.
Frames with wrong dimensions, payload length or an extra flip flag are rejected.
The backend clears stale output and clears on shutdown.

## Package boundaries

The original osf_receiver_pkg, proto_panel_display_pkg, opencv2_display_pkg and
param_server_pkg are restored to commit 6110aa90568b796e18188dd3104e273d1e30c9f0.
All pre-existing package trees match that revision. Prootface-specific executables
are installed only by prootface_pkg: prootface, osf_input, atlas_display and
preview_display. The launch does not start or reconfigure the legacy parameter
server or display backends. Preview dimensions are local parameters.

An existing OSFData publisher can be reused with start_receiver:=false. The
udp_host/udp_port arguments apply only to our optional input adapter; they do not
change the original receiver. Existing receiver behavior is preserved. Datagram
validation, multi-face delivery and clean socket shutdown described by this
package belong to its own optional input adapter.

Future changes to existing packages are limited to adding parameters in
param_server_pkg and adding new interface definitions in interface_pkg. Other
existing package code, dependencies and interfaces must stay unchanged.

## Ownership, scheduling and failure behavior

The face node asks for control once on startup using its fully qualified name.
It publishes that sender on `request_screen_display`; the existing dispatcher
filters producers and forwards selected frames to `screen_display`. Switching
ownership to another producer is respected. The current dispatcher has no timed
ownership lease or automatic crash failover.

Input queues retain one multi-face datagram before filtering by face ID.
The input callback retains valid selected-face measurements and latches contact.
The steady 24 Hz controller runs independently of rendering. EGL belongs to a
worker thread with at most one pending pose and one completed frame. Newer poses
replace queued older ones; completed frames older than 250 ms are discarded.
Tracking loss clears queued contact and eases the character toward rest. A renderer failure is surfaced
instead of silently replaying old frames.

The legacy `facial_expression` named-PNG command interface is not mapped to
continuous articulation. Manual/demo controls remain available through the shared
UI. OpenSeeFace/Hailo startup and model deployment stay upstream of this package.

## Validation

The integration workflow builds real interface messages and three ROS packages
under Jazzy. It runs core/LED-map tests, then a UDP-to-ROS-to-EGL-to-dispatcher test
with recorded driver bytes and a real installed-launch smoke test. Calibration
HTTP, profile saving, ownership changes, malformed datagrams and multi-face input
are included. Fixtures are original synthetic geometry; no private character or
recording is uploaded. The driver is a stub in CI, so physical output is not claimed.

```sh
python3 -m unittest discover -s src/prootface_pkg/test -v
```

Actual camera placement, wearer calibration, CM4 latency and physical LEDs still
require device testing with the actual map and installed driver.
