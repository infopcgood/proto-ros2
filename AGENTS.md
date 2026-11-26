# Working instructions

- Preserve existing nodes and display ownership unless a task changes them.
- User boundary: do not modify existing packages. The only permitted extensions
  are adding parameters in param_server_pkg and adding new interface definitions
  in interface_pkg; do not rewrite their existing behavior or definitions.
- Keep all Prootface adapters, optional input/display nodes and tests inside the
  new prootface_pkg. Do not add files or dependencies to existing packages.
- Prootface integration uses the existing OSF receiver, continuous calibrated
  animation, a headless EGL renderer and one synchronized left/right atlas.
- Private character assets, textures, compiled meshes and source-specific import
  settings must never enter this public repository or CI artifacts.
- Keep input/control at 24 Hz; presentation may hold poses at 24/12/6 fps.
  Use monotonic freshness and a latest-only render handoff. No future phonemes.
- Existing single-image backends remain available. The dense WS atlas backend
  requires the actual LED map and a matching driver strip count; do not invent
  panel density, geometry, strip counts or wiring for the user.
- Source-only/mocked tests are not ROS execution. Exercise the real messages,
  dispatcher and node under ROS Jazzy CI; distinguish desktop Mesa results from
  CM4 and physical LED measurements.
- Update integration docs/status at milestones and publish using expected head
  SHAs. Never force-push over concurrent changes.
