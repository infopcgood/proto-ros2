# desktop_streaming_pkg

Capture a computer's desktop and send it to the existing proto-ros2 displays.
Run this node **on the computer whose desktop you want to capture**. The
parameter server, screen dispatcher, and display backend may run on a different
computer, such as the Raspberry Pi, over the ROS 2 network.

Frames are resized to the parameter server's `DISPLAY_WIDTH` and `DISPLAY_HEIGHT`
before transmission. The node requests screen ownership via
`request_screen_control`, then publishes packed BGR `interface_pkg/msg/Image`
messages on `request_screen_display`. The dispatcher forwards the selected
producer's frames to `screen_display`. This captures video only, without audio.

## Requirements and build

- A ROS 2 environment compatible with the receiver (the project uses Jazzy).
- An active graphical desktop. On Linux, use an **X11/Xorg session** with a valid
  `DISPLAY` and access to that user's X server. Native Wayland capture is not
  supported. Launch from a terminal in the desktop session, not a headless SSH
  session. Do not disable X server access controls.
- [MSS](https://python-mss.readthedocs.io/v10.2.0/usage.html) 10.2.x and Pillow
  9.1 or later. MSS also supports Windows and macOS; the computer still needs a
  working ROS 2 installation, and macOS requires Screen Recording permission
  for the terminal/Python process. These platforms have not been smoke-tested
  for this package.

Use the Python environment that your ROS 2 installation uses. If your OS manages
system Python, use a ROS-compatible virtual environment with system packages
visible, as described in the repository's setup instructions. From the workspace:

```bash
python3 -m pip install 'mss>=10.2,<11' 'Pillow>=9.1'
rosdep install --from-paths src/interface_pkg src/desktop_streaming_pkg \
  --ignore-src --skip-keys python3-mss-pip -y
colcon build --packages-up-to desktop_streaming_pkg
source install/setup.bash
```

The rosdep MSS key is skipped above because MSS was installed into the selected
Python environment explicitly. Building this subset does not require the Pi's
LED or sensor libraries. Build the receiver packages separately using the main
repository's instructions.

## Run

Source ROS 2 and the workspace in every terminal. For separate desktop and Pi
machines, use the same `ROS_DOMAIN_ID` on both, enable subnet discovery, and
ensure the network/firewall allows ROS 2 DDS discovery and traffic:

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
```

Start the receiver nodes in separate terminals:

```bash
# Terminal 1: provides display dimensions (defaults to 50 x 34).
ros2 run param_server_pkg param_server

# Terminal 2: arbitrates screen ownership.
ros2 run screen_dispatcher_pkg screen_dispatcher

# Terminal 3: preview, or substitute your configured hardware display backend.
ros2 run opencv2_display_pkg opencv2_display
```

For HUB75 panels, configure the parameter server with the dimensions expected
by your hardware, for example `--ros-args -p DISPLAY_WIDTH:=64 -p DISPLAY_HEIGHT:=32`.
The proto-panel backend requires 50 x 34. Existing receiver nodes require their
own runtime dependencies, including OpenCV/NumPy for the preview.

On the desktop computer, start capture after the receiver services are available:

```bash
ros2 run desktop_streaming_pkg desktop_streaming --ros-args \
  -p monitor:=1 -p fps:=24.0
```

To capture all monitors or stretch the desktop to fill the display:

```bash
ros2 run desktop_streaming_pkg desktop_streaming --ros-args \
  -p monitor:=0 -p fps:=15.0 -p preserve_aspect_ratio:=false
```

The package is started manually; `launch.sh` does not start the required
parameter server or display backend. No changes to that script are needed to
run a desktop producer on a separate computer.

## Parameters

All parameters are read-only after startup; restart the node to change them.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `monitor` | `1` | MSS monitor index: `1..N` for an individual monitor, `0` for the combined desktop. The first monitor is not necessarily the OS primary monitor. |
| `fps` | `24.0` | Maximum requested capture rate; use a positive floating-point value. Actual throughput depends on capture, resizing, and transport. |
| `preserve_aspect_ratio` | `true` | Fit the entire desktop with centered black padding; `false` stretches it to the display size. |
| `flip_second_screen` | `false` | Forward the existing image flag used by the HUB75 backend. |
| `service_timeout` | `10.0` | Positive timeout in seconds for each service discovery wait and each response wait. |

Display dimensions are fetched once at startup. Invalid dimensions, an
unavailable monitor, a capture failure, an unavailable service, or rejected
screen control cause the node to exit with an error. Capture is checked before
requesting screen ownership, and capture resources are closed on exit.

Only the most recently requested producer controls the dispatcher. Starting
another producer can take over the screen while this node continues capturing.
Stopping capture with Ctrl+C does not restore the previous owner: the existing
dispatcher has no release/restore service. To hand control back to a running
face renderer, use:

```bash
ros2 service call /request_screen_control interface_pkg/srv/ControlRequest \
  "{sender: 'face_renderer_node'}"
```

## Tests

The frame tests use real Pillow conversions. The node contract tests replace ROS
and capture APIs with test doubles, so they can run without ROS, a desktop, or
LED hardware. They do not validate DDS transport or native desktop capture.

```bash
PYTHONPATH=src/desktop_streaming_pkg python3 -m unittest discover \
  -s src/desktop_streaming_pkg/test -v
```

In a ROS workspace, they also run with `colcon test --packages-select
desktop_streaming_pkg` after building. To smoke-test the complete path, start
the receiver and capture commands above, verify the desktop appears in the
OpenCV preview, switch monitors, and stop capture with Ctrl+C.
