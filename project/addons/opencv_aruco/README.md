# OpenCV ArUco Marker Tracking (Godot addon)

ArUco marker tracking for Godot 4.7+ that is a **drop-in replacement for Godot's built-in
OpenXR marker tracking** (`XR_EXT_spatial_marker_tracking` / spatial entities). Detection runs
through OpenCV on frames pulled from the Godot `CameraServer` (Quest passthrough cameras on
Android, webcam on desktop), but the results surface through the **standard Godot XR route**:
one genuine `OpenXRMarkerTracker` per marker, registered with the `XRServer`, consumed with
`XRAnchor3D` — exactly like the engine's own backend and the official
["OpenXR spatial entities" tutorial](https://docs.godotengine.org/en/latest/tutorials/xr/openxr_spatial_entities.html).

Why: the Meta Quest OpenXR runtime currently only reports **QR codes** through the spatial
entities marker route — no ArUco. This addon fills that gap without inventing a new API:
consumer code written against the standard route keeps working unchanged, and if a future
runtime ships native ArUco you can switch back by enabling Godot's built-in marker tracking
and removing the `ArucoMarkerTracking` node.

## Contents

- `aruco_marker_tracking.gd` — the runtime node (`ArucoMarkerTracking`), the whole pipeline:
  camera feed selection, GPU→CPU readback, worker-thread OpenCV detection, capture-latency
  compensation, tracker publication.
- `bin/` — the `opencv_aruco` GDExtension (OpenCV statically linked; `OpenCVProcessor` class).
  Prebuilt for Android arm64 (Quest); other platforms are built with `scons` (see the repo
  README).
- `plugin.cfg` / `plugin.gd` / `export_check.gd` — editor plugin with the Android export guard:
  fails the export (and deletes the broken APK) when a required `.so` is missing, instead of
  letting Godot silently package a 0-byte library.
- `examples/marker_tracking_example.gd` — one attachable script showing both consumption
  routes (the standard XR route and the id-keyed API) with comments; copy what you need.

## Setup

1. Get the addon into your project — either grab the prebuilt zip (built by the
   `Build addon zip` GitHub Actions workflow; attached to GitHub releases on tags, or as a
   workflow artifact) and extract it into the project root (it unpacks as
   `addons/opencv_aruco/` with binaries for Windows x86_64, Linux x86_64, macOS arm64 and
   Android arm64, debug + release), or copy `addons/opencv_aruco/` from this repo and build
   the binaries yourself. Then enable the plugin
   (Project → Project Settings → Plugins → "OpenCV ArUco Marker Tracking").
2. Add an **ArucoMarkerTracking** node anywhere in your XR scene (it has no scene-tree
   dependencies) and configure its exports: marker sizes, camera intrinsics/distortion, capture
   latency. The defaults are calibrated for the Quest 3 left passthrough camera ("50", 640×480).
3. Android export: enable the CAMERA permission plus the custom permission
   `horizonos.permission.HEADSET_CAMERA` (Quest passthrough camera access).
4. Markers are DICT_4X4_50 (ids 0–49); the dictionary is baked into the C++ detector.

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
| `marker_id` | the ArUco id (0–49) |
| `bounds_size` | the marker's configured physical size (meters) |
| Momentary loss | after `marker_lost_timeout_ms` (default 500 ms): `invalidate_pose` + state `PAUSED`, tracker **kept** (consumers hold the last pose; `show_when_tracked` hides) |
| Long absence | after `marker_stopped_timeout_s` (default 10 s, 0 = never): state `STOPPED`, then `XRServer.remove_tracker` |
| Registration order | `add_tracker` only after id/type/bounds/pose are set, so `tracker_added` handlers always see a complete tracker |

`get_entity()` returns an invalid RID — there is no OpenXR spatial entity behind these
trackers. Code that goes through the low-level spatial entity snapshot API instead of the
tracker surface is talking to the runtime directly and cannot be intercepted.

## Direct (id-keyed) API

For consumers that want poses without tracker objects, the node also offers:
`get_marker_pose(id)` / `get_marker_world_pose(id)`, `has_marker(id)`, `marker_age_ms(id)`,
`markers_fresh(ids)`, `markers_ever_seen(ids)`, `get_average_marker_pose(ids)`,
`get_marker_tracker(id)`, `get_tracker_name_for(id)`, the `markers_updated(ids)` signal, and
`get_camera_texture()` / `camera_feed_started` for a debug feed preview. Capability-style
checks (`is_aruco_supported()` → true, `is_qrcode_supported()` → false, …) mirror
`OpenXRSpatialMarkerTrackingCapability` for feature-gating code.

## Coexistence with the built-in backend

Running this addon alongside Godot's own marker tracking
(`xr/openxr/extensions/spatial_entity/enabled` + `enable_builtin_marker_tracking`) is possible
— names cannot collide — but you would get two anchor trackers per physical marker if the
runtime ever learns ArUco. The node prints a warning at startup when both are enabled.
