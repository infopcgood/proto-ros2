# ROS runtime API

The Python animation and HTTP interfaces are documented in the separate
[core API reference](https://github.com/infopcgood/prootface/blob/master/docs/API.md).
This adapter does not run inference, invent phoneme labels, or start OpenSeeFace.

## Nodes, topics and ownership

Topic/service names below are relative to the ROS namespace.

| Interface | Type | Direction / behavior |
| --- | --- | --- |
| `osf_data` (configurable `input_topic`) | `interface_pkg/msg/OSFData` | Face node subscribes; selects `face_id`, validates data, uses local monotonic receipt time. |
| `facial_expression` (configurable `expression_topic`) | `interface_pkg/msg/FacialExpression` | External button selection of expression banks; independent of speech input. |
| `request_screen_display` | `interface_pkg/msg/Image` | Face node publishes completed two-view atlases after its startup control grant. |
| `request_screen_control` | `interface_pkg/srv/ControlRequest` | Face node requests screen ownership once using its fully qualified node name. |
| `screen_display` | `interface_pkg/msg/Image` | Existing dispatcher forwards frames matching its `dominating_node_name`. OpenCV or atlas WS backend subscribes. |

`ControlRequest` request: `string sender`. Response: `bool success`,
`string message`. A successful request changes the dispatcher's active sender.
The face node retries unavailable/failed service calls; a completed denial ends
its attempt. It does not periodically reclaim ownership from another producer.
There is no timed ownership lease or crash failover.

DDS uses reliable, volatile, keep-last defaults. The optional Prootface input publisher
and face input subscriber each have depth 39 so a complete multi-face datagram
is retained until face filtering. This is a bounded delivery queue, not a timed
animation backlog. The application retains only the latest valid selected-face
measurement. The face image publisher and WS subscriber have depth 1. The
existing dispatcher retains its depth-10 image queues.

## Image payload contract

This is the project's custom `Image`, not `sensor_msgs/Image`:

| Field | Value |
| --- | --- |
| `sender` (`string`) | Fully qualified producer node name, matching the ownership request |
| `height` (`uint32`) | One side's virtual height, H |
| `width` (`uint32`) | Twice one side's virtual width, 2W |
| `flip_second_screen` (`bool`) | Always `false` for independently projected atlases |
| `data` (`uint8[]`) | Exactly `H * 2W * 3` bytes: row-major, interleaved BGR, top-left origin, no padding |

Columns `[0, W)` are anatomical left; `[W, 2W)` are anatomical right. Do not add
a second mirror. Physical orientation belongs in the LED map. There is no ROS
header, frame timestamp or encoding field in this message. The producer rejects
render completions older than 250 ms before publishing; downstream transport
cannot recheck age from this message alone.

Control uses a steady 24 Hz timer. Presentation submits every 1, 2 or 4 ticks
for 24, 12 or 6 fps; slower rendering drops superseded pending poses. Rates are
target cadences, not a hardware throughput guarantee. EGL is owned by one worker
thread. Tracking expires after 250 ms and eases to rest; old queued closures are
cleared. Pausing ROS simulated time does not pause control or the WS watchdog.

## Face node parameters

Run directly with `ros2 run prootface_pkg prootface --ros-args -p ...`.
Node name: `prootface_node`. All adapter parameters below are **read-only after
startup**; restart the node to change them. Runtime controls/style/calibration
use the shared HTTP API instead. Standard ROS parameters are separate.

| Parameter | Type / default | Meaning |
| --- | --- | --- |
| `character_path` | string, `""` (required) | Existing private schema-2 JSON with matching PNG sidecar |
| `profile_path` | string, `~/.local/share/prootface/calibration.json` | Load/save wearer profile; defaults used if absent |
| `style_path` | string, `""` | Optional JSON of supported style fields; loaded once |
| `face_id` | integer, `0` | Nonnegative tracked face ID |
| `input_topic` | string, `osf_data` | OSFData subscription topic |
| `expression_topic` | string, `facial_expression` | External expression-command topic |
| `expression_transition` | float, `0.2` | Bank crossfade duration in seconds, 0–2 |
| `side_width` | integer, `256` | Per-view virtual width, 1–4096 |
| `side_height` | integer, `144` | Per-view virtual height, 1–4096 |
| `presentation_fps` | integer, `24` | Exactly 24, 12 or 6; controller remains 24 Hz |
| `egl_platform` | string, `auto` | `auto`, `surfaceless`, `default` |
| `web_host` | string, `127.0.0.1` | HTTP bind address |
| `web_port` | integer, `8080` | Negative disables HTTP; 0 selects an available port |

HTTP defaults to loopback and has no authentication. The node hosts the same
`/api/state`, `/api/events`, `/api/control`, `/api/style`, `/api/calibration/*`,
and character endpoints as the core; it does not bind a UDP socket. Successful
HTTP commands are reflected on the next controller tick. Style changes are not
automatically persisted. Calibration saving writes `profile_path` atomically.

## Launch arguments

```sh
ros2 launch prootface_pkg face.launch.py \
  character:=/path/to/private/character.json \
  profile:=/path/to/private/calibration.json \
  presentation_fps:=12 backend:=opencv
```

| Launch argument | Default | Effect |
| --- | --- | --- |
| `character`, `profile`, `style` | `""`, default profile path above, `""` | Map to node `*_path` parameters |
| `side_width`, `side_height`, `presentation_fps` | `256`, `144`, `24` | Rendering and presentation |
| `face_id`, `input_topic` | `0`, `osf_data` | Selected face and optional input-adapter remapping |
| `expression_topic`, `expression_transition` | `facial_expression`, `0.2` | Button input and transition duration |
| `web_host`, `web_port`, `egl_platform` | Same as node | HTTP and graphics |
| `backend` | `opencv` | `opencv`, `ws`, or `none` |
| `start_receiver` | `true` | Start prootface_pkg/osf_input |
| `udp_host`, `udp_port` | `127.0.0.1`, `11573` | Prootface input bind address; port 0 is available for tests |
| `panel_map` | `""` | Required actual coordinate-map JSON for `ws` |
| `driver_strips` | `8` | Must match map and installed smileds build; does not configure driver strip count |
| `brightness` | `3` | Driver-specific integer passed to smileds |

Launch starts the existing dispatcher without changing its package. The optional
input, OpenCV preview (scale 2) and WS atlas nodes all belong to prootface_pkg.
The legacy parameter server and display nodes are not started or reconfigured.
`backend:=none` still runs the publisher and dispatcher.
If the receiver already exists, use `start_receiver:=false` and ensure its topic
matches `input_topic`. Do not start a second UDP listener on the same port.

## Atlas WS backend and map API

Executable: `ros2 run prootface_pkg atlas_display`.
Node: `prootface_atlas_display_node`. Startup-only/read-only parameters:

| Parameter | Default | Contract |
| --- | --- | --- |
| `panel_map` | `""` | Required schema-1 map path |
| `driver_strips` | `8` | Installed driver's actual strip count |
| `brightness` | `3` | Driver brightness value |
| `stale_timeout` | `1.0` | Finite positive seconds since last accepted frame before clearing |
| `expected_side_width`, `expected_side_height` | `0`, `0` | Nonzero values must match map; launch supplies renderer dimensions |

The watchdog runs on a steady timer, checks every 100 ms, and clears after the
first accepted frame becomes stale. It also clears on shutdown. Wrong dimensions,
byte count or an extra mirror flag reject a frame and do not refresh the watchdog.

`prootface_pkg.atlas_mapping.AtlasMap(configuration)` and
`AtlasMap.load(path)` are ROS-independent Python APIs requiring NumPy.
`rgb_bytes(data, width, height, flip_second_screen=False) -> bytes` samples the
BGR atlas into RGB physical-driver order. Data is a byte buffer or uint8 list.
Map schema:

| Key | Required value |
| --- | --- |
| `schema` | `1` |
| `side_width`, `side_height` | Positive integer virtual dimensions |
| `leds_per_strip`, `num_strips` | Positive integers matching the physical driver |
| `pixels` | Exactly `leds_per_strip * num_strips` entries, strip-major then LED index |

Each entry is `["left", x, y]`, `["right", x, y]`, or `null` (black output).
Coordinates are integer side-local pixels with top-left origin. The map encodes
rotation and wiring; no physical geometry is inferred. Schema/dimension/coordinate
errors raise `ValueError`; file/JSON operations can also raise their usual errors.
The legacy display executable and named-PNG face interface remain separate.

## Package-owned optional input and preview

`ros2 run prootface_pkg osf_input` starts `prootface_osf_input_node`. Its read-only
parameters are `udp_host` (127.0.0.1) and `udp_port` (11573; zero selects a test
port). It publishes OSFData on osf_data; standard ROS remapping selects another
topic. Start only one UDP listener per address/port. Set launch start_receiver
false when the existing unmodified receiver already publishes the desired topic.

`ros2 run prootface_pkg preview_display` starts `prootface_preview_node` and
subscribes to screen_display. Read-only parameters: scale=2, expected_width=0,
expected_height=0. Zero accepts the incoming dimension; launch sets explicit
atlas dimensions. The node validates BGR byte count, rejects an extra mirror,
and owns its OpenCV window without using the global parameter server.

All pre-existing packages retain their code from before this integration. Only
new parameters in param_server_pkg and new definitions in interface_pkg may be
added to existing packages in future work; do not alter their existing behavior.

## External expression commands

Use the existing interface_pkg/msg/FacialExpression fields: type selects emotion
(or face) for the whole face, or eyes/mouth for one channel; expression is a
compiled bank name; override_time is zero for a latched selection or positive
seconds for a temporary override returning to the latched choice. Default banks
are neutral, happy, angry, sad and unamused when supplied by the character. idle
and smile alias neutral and happy. Invalid/unavailable commands are rejected
without changing the current selection and increment expression_rejected on the
node. No existing interface definition or package is modified.

HTTP provides the same selection at POST /api/expression with expression, optional
part (face/eyes/mouth), hold and transition fields. Snapshot expression reports
available banks and per-part selected/latched names, weights and monotonic expiry.
Zero-duration holds persist through speech and tracking loss until changed or
restarted. Duplicate commands do not restart transitions or extend holds.
The detailed geometry/composition contract is in the core
[expression reference](https://github.com/infopcgood/prootface/blob/master/docs/EXPRESSIONS.md).
