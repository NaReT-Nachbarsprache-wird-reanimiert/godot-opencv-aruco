# OpenCV ArUco Marker Tracking (Godot addon)

ArUco marker tracking for **Godot 4.7+** that is a **drop-in replacement for Godot's built-in
OpenXR marker tracking** (`XR_EXT_spatial_marker_tracking` / spatial entities). Detection runs
through OpenCV on camera frames, but the results surface through the **standard Godot XR route**:
one genuine `OpenXRMarkerTracker` per marker, registered with the `XRServer`, consumed with
`XRAnchor3D` — exactly like the engine's own backend and the official
["OpenXR spatial entities" tutorial](https://docs.godotengine.org/en/latest/tutorials/xr/openxr_spatial_entities.html).

Why: the Meta Quest OpenXR runtime currently only reports **QR codes** through the spatial
entities marker route — no ArUco. This addon fills that gap without inventing a new API:
consumer code written against the standard route keeps working unchanged, and if a future
runtime ships native ArUco you can switch back by enabling Godot's built-in marker tracking
and removing the `ArucoMarkerTracking` node.

> **Godot 4.7 is a hard requirement**, not a recommendation. `OpenXRMarkerTracker`,
> `OpenXRSpatialEntityTracker` and `OpenXRSpatialComponentMarkerList` are 4.7 classes and this
> addon names all three. They are also marked *experimental* upstream, so a future 4.x may move
> them. `bin/opencv_aruco.gdextension` declares `compatibility_minimum = "4.7"` so an older
> project fails at load rather than at script parse.

## Contents

- `aruco_marker_tracking.gd` — the runtime node (`ArucoMarkerTracking`): camera feed, worker-thread
  OpenCV detection, capture-time head pose, tracker publication.
- `bin/` — the `opencv_aruco` GDExtension (OpenCV statically linked; the `OpenCVProcessor` class).
- `plugin.cfg` / `plugin.gd` / `export_check.gd` — editor plugin with the Android export guard:
  fails the export (and deletes the broken APK) when a required `.so` is missing, instead of
  letting Godot silently package a 0-byte library.
- `examples/marker_tracking_example.gd` — one attachable script showing both consumption routes.

**Two sibling addons ship in the same zip and are not optional in practice:**

| Addon | What you lose without it |
| --- | --- |
| `GodotAndroidCamera` | The Quest path falls back from CameraX (raw Y plane on the CPU, with each frame's **sensor timestamp**) to a CameraServer readback corrected by a fixed `camera_latency_ms` guess. Markers swim more under head motion. |
| `CameraServerExtension` | No webcam feed on Windows desktop at all. |

Both are reached defensively at runtime — the first is `load()`ed by path, the second through
`ClassDB` — so a missing one degrades quietly rather than erroring. That is exactly why they are
bundled: you would not otherwise find out.

## Setup

1. Extract the release zip into your project root. It unpacks as `addons/opencv_aruco/`,
   `addons/GodotAndroidCamera/` and `addons/CameraServerExtension/`, with binaries for
   Windows x86_64, Linux x86_64, macOS arm64 and Android arm64 (debug + release). Then enable
   **OpenCV ArUco Marker Tracking** and **GodotAndroidCamera** under
   Project → Project Settings → Plugins.
2. Add an **ArucoMarkerTracking** node anywhere in your XR scene — it has no scene-tree
   dependencies — and set `marker_dictionary` and the marker sizes (below).
3. Android export: enable the CAMERA permission plus the custom permission
   `horizonos.permission.HEADSET_CAMERA` (Quest passthrough camera access), arm64-v8a, min SDK 24.

## Configuring it for YOUR markers

Two settings are about physical objects, and both are silent when wrong:

- **`marker_dictionary`** — `ArUco MIP 36h12` (default) or `4x4 (50 ids)`. It must match what your
  markers are printed in; ids collide between dictionaries, so the wrong choice does not error, it
  just detects nothing (or, worse, something else). 36h12 is the better code — 36 bits at a minimum
  Hamming distance of 12, against 16 bits at distance 4 — and the detector runs with no error
  correction, which is what makes 4x4_50 flaky here. 4x4_50 exists because printed markers cannot be
  recompiled. `tools/generate_markers.py` in the provider repo emits either.
- **`default_marker_size` / `marker_sizes`** — the physical side length of the **black square**, in
  meters, per marker id. This is a pure metric-scale knob: `solvePnP` reads a marker measured 5 mm
  too small as ~7% further away. Measure, do not guess.

The camera calibration (`camera_intrinsics`, `camera_distortion`, `lens_rotation_raw`,
`lens_translation`) is measured on a Quest 3's left passthrough camera and should carry over to
another Quest 3. Off Android the intrinsics are replaced at runtime with a crude pinhole guess and
a loud warning — enough to exercise detection and tracker publication on a desktop, not enough to
trust a distance. If poses are visibly wrong on your device, the measurement tooling (reprojection
overlay, hand-eye solve, plot scripts) lives in the provider repo's own demo project rather than in
this addon; clone it and follow its CLAUDE.md.

## Consuming markers (the standard XR route)

The same manager pattern the official spatial entities tutorial uses — nothing in it is
specific to this addon:

```gdscript
func _ready() -> void:
    XRServer.tracker_added.connect(_on_tracker_added)
    XRServer.tracker_removed.connect(_on_tracker_removed)

func _on_tracker_added(tracker_name: StringName, type: int) -> void:
    if type != XRServer.TRACKER_ANCHOR:
        return
    var tracker := XRServer.get_tracker(tracker_name)
    if not tracker is OpenXRMarkerTracker:
        return
    if tracker.marker_type == OpenXRSpatialComponentMarkerList.MARKER_TYPE_ARUCO:
        var anchor := XRAnchor3D.new()          # or instantiate your marker scene
        anchor.tracker = tracker_name           # pose defaults to "default"
        anchor.show_when_tracked = true
        xr_origin.add_child(anchor)             # anchors are children of the XROrigin3D
        print("ArUco id %d, size %s" % [tracker.marker_id, tracker.bounds_size])
```

Because the tracker names are deterministic (see below), you can ALSO pre-author an
`XRAnchor3D` in a scene and set its `tracker` property to e.g.
`openxr/spatial_entity/aruco_3` — something the real OpenXR backend cannot offer (its entity
ids are runtime-assigned).

## The tracker contract (mirrors the engine's marker capability)

| Aspect | Value |
| --- | --- |
| Tracker class | `OpenXRMarkerTracker` (the engine class, so `is OpenXRMarkerTracker` checks work) |
| Tracker name | `openxr/spatial_entity/aruco_<marker_id>` (upstream prefix, deterministic suffix) |
| Tracker type | `XRServer.TRACKER_ANCHOR` |
| Pose | name `"default"`, transform in the XR **play space**, unscaled, zero velocities, confidence HIGH — `XRNode3D` applies world scale + reference frame itself |
| `marker_type` | `OpenXRSpatialComponentMarkerList.MARKER_TYPE_ARUCO` |
| `marker_id` | the ArUco id (0–49 for 4x4_50, 0–249 for 36h12) |
| `bounds_size` | the marker's configured physical size (meters) |
| Momentary loss | after `marker_lost_timeout_ms` (default 500 ms): `invalidate_pose` + state `PAUSED`, tracker **kept** (consumers hold the last pose; `show_when_tracked` hides) |
| Long absence | after `marker_stopped_timeout_s` (default 10 s, 0 = never): state `STOPPED`, then `XRServer.remove_tracker` |
| Registration order | `add_tracker` only after id/type/bounds/pose are set, so `tracker_added` handlers always see a complete tracker |

`get_entity()` returns an invalid RID — there is no OpenXR spatial entity behind these
trackers. Code that goes through the low-level spatial entity snapshot API instead of the
tracker surface is talking to the runtime directly and cannot be intercepted.

Publication happens on the main thread, but not necessarily inside `_process`: on the CameraX path
a finished detection is collected from the camera callback, so `tracker_added` and
`markers_updated` can fire before your node's own `_process` for that frame.

## Direct (id-keyed) API

For consumers that want poses without tracker objects:
`get_marker_pose(id)` / `get_marker_world_pose(id)`, `has_marker(id)`, `marker_age_ms(id)`,
`markers_fresh(ids)`, `markers_ever_seen(ids)`, `get_average_marker_pose(ids)`,
`get_marker_size(id)`, `get_marker_tracker(id)`, `get_tracker_name_for(id)`, the
`markers_updated(ids)` signal, and `get_camera_texture()` / `camera_feed_started` for a debug feed
preview. Capability-style checks (`is_aruco_supported()` → true, `is_qrcode_supported()` → false, …)
mirror `OpenXRSpatialMarkerTrackingCapability` for feature-gating code.

> **`get_marker_pose()` is PLAY space; `get_marker_world_pose()` is WORLD space.** They differ by
> the `XROrigin3D` transform and the XR reference frame. Code ported from an older, pre-addon
> version of this pipeline — where `get_marker_pose()` returned world space — will compile, run,
> and be wrong by exactly that amount. This is the one API change worth grepping for.

## Hooks for external tooling

`get_processor()` exposes the `OpenCVProcessor` for code that needs `project_marker_corners()` or
`get_lens_pose()`, and three signals — `detection_applied`, `frame_sampled`, `xr_locator_ready` —
carry everything a measurement rig needs. The provider repo's diagnostics nodes attach through
exactly these. Nothing in the addon depends on them; with no listeners they cost nothing.

## Coexistence with the built-in backend

Running this addon alongside Godot's own marker tracking
(`xr/openxr/extensions/spatial_entity/enabled` + `enable_builtin_marker_tracking`) is possible
— names cannot collide — but you would get two anchor trackers per physical marker if the
runtime ever learns ArUco. The node prints a warning at startup when both are enabled.
