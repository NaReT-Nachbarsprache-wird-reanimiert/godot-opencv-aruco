# Drop-in replacement for Godot's built-in OpenXR marker tracking (XR_EXT_spatial_marker_tracking),
# backed by the opencv_aruco GDExtension and a camera feed instead of the OpenXR runtime.
#
# Why it exists: the Meta Quest OpenXR runtime only reports QR codes through the spatial entities
# marker route (no ArUco). This node produces the SAME Godot-facing surface Godot's own
# OpenXRSpatialMarkerTrackingCapability would, so consumer code written against the standard route
# keeps working unchanged:
#   - one genuine OpenXRMarkerTracker per detected marker, registered via XRServer.add_tracker()
#     AFTER it is fully populated (so a tracker_added handler sees marker_id/type/bounds/pose),
#   - tracker type TRACKER_ANCHOR (set by the OpenXRSpatialEntityTracker constructor),
#   - pose name "default", transform in the XR play space, unscaled and WITHOUT the reference
#     frame applied (XRNode3D/XRAnchor3D apply world scale + reference frame themselves through
#     XRPose.get_adjusted_transform, exactly as they do for the engine's own trackers),
#   - marker_type = MARKER_TYPE_ARUCO, marker_id = the ArUco id, bounds_size = physical size,
#   - momentary loss pauses the tracker (invalidate_pose + ENTITY_TRACKING_STATE_PAUSED, tracker
#     KEPT so consumers hold the last pose), long absence removes it (STOPPED + remove_tracker) --
#     both mirroring OpenXRSpatialMarkerTrackingCapability::_process_snapshot in the engine.
#
# One deliberate difference: the engine names entity trackers "openxr/spatial_entity/<entity_id>"
# with an OPAQUE runtime-assigned id, so scenes can never pre-author an XRAnchor3D by name even on
# the real backend -- the documented pattern is signal-driven (XRServer.tracker_added). Our names
# are "openxr/spatial_entity/aruco_<marker_id>": they keep the upstream prefix (prefix-filtering
# consumers still match), can never collide with the runtime's numeric entity ids, and as a bonus
# are DETERMINISTIC, so an XRAnchor3D bound to a known marker id can be authored in a scene.
#
# TWO CAMERA BACKENDS, picked in _ready and never mixed:
#   PUSH (Quest, preferred) -- the GodotAndroidCamera plugin (CameraX ImageAnalysis) hands us the
#     raw Y plane on the CPU together with each frame's SENSOR TIMESTAMP. No GPU->CPU readback, and
#     the head pose can be looked up at the frame's true exposure time.
#   PULL (desktop, and Quest without that plugin) -- CameraServer + CameraTexture.get_image(). No
#     timestamp exists on this path, so camera_latency_ms is the only correction available.
# The plugin is an OPTIONAL dependency: it is reached through load() into untyped vars, never a
# typed AndroidCamera reference, because a typed reference to another addon's class_name is a PARSE
# error in any project that does not install it -- which would take this whole script down with it.
#
# EVERYTHING IS PLAY SPACE. The C++ side is frame-agnostic: detect_and_solve_all computes
# head_pose * lens_pose and returns marker poses in whatever space its head_pose argument was in.
# We hand it a play-space head pose (XRServer's "head" tracker, or OpenXRHeadLocator, which returns
# play space natively), so the results are play space and go onto trackers verbatim. World space
# exists only where a consumer asks for it: get_marker_world_pose().
#
# The measurement apparatus (hand-eye capture, reprojection overlay, TCP frame streamer, OpenXR
# timing checks) is NOT part of this addon and is not shipped with it. It lives in the provider
# repo's own demo project and attaches through the three signals below plus get_processor(); with
# nothing connected they cost nothing.
#
# Usage: add this node anywhere in the scene (it has no scene-tree dependencies), configure the
# exports, consume markers via XRServer.tracker_added / XRAnchor3D as in the official
# "OpenXR spatial entities" tutorial. See res://addons/opencv_aruco/README.md.
class_name ArucoMarkerTracking
extends Node

# Tracker name prefix; full name = PREFIX + str(marker_id). Kept in sync with the class comment.
const TRACKER_NAME_PREFIX := "openxr/spatial_entity/aruco_"

# The optional CameraX plugin, reached by PATH rather than by class_name. See the class comment.
const ANDROID_CAMERA_SCRIPT := "res://addons/GodotAndroidCamera/android_camera.gd"
# AndroidCamera.OutputFormat.LUMA -- the camera's native Y plane, 1 byte per pixel, which is
# exactly the grayscale the C++ detector consumes (RGBA would cost a CPU conversion per frame).
# Spelled out rather than read off the loaded script: this is the one value we need from that
# addon's enum, and a literal cannot fail to resolve on a device we cannot debug from here.
const ANDROID_CAM_FORMAT_LUMA := 0

## Emitted after a detection result was applied, with the ArUco ids seen in that frame. For
## one-shot reactions; polling the getters below from _process is equally fine.
signal markers_updated(ids: Array)
## Emitted once the camera feed is running and frames will start arriving. Texture2D rather than
## CameraTexture because the two backends produce different things: the pull path emits the live
## CameraTexture, the push path an ImageTexture it updates per frame. get_camera_texture() serves
## late subscribers.
signal camera_feed_started(texture: Texture2D)

# --- Diagnostics hooks ------------------------------------------------------------------------
# The three signals the demo-only measurement nodes attach to. They are signals rather than typed
# child nodes for the same reason the CameraX plugin is loaded by path: a typed
# `@onready var _d: DetectionDiagnostics` inside the addon would be a parse error wherever those
# demo scripts are absent. Every type named here is an engine or extension class, so this file
# stays self-contained. A signal with no listeners costs nothing.

## Once per finished detection, from _poll_detection_task -- AFTER the result has been applied and
## BEFORE any pending frame is dispatched (that dispatch overwrites _detecting_img). The image
## travels as an ARGUMENT, which is what makes that ordering structural instead of a comment.
## poses are PLAY space, head_pose is the pose they were baked with -- same space, which is all
## project_marker_corners() needs to reproject them.
signal detection_applied(image: Image, corners: Dictionary, poses: Dictionary, head_pose: Transform3D)
## Once per _process. Argument list mirrors what the timing checks need; xr_clock_offset_ns is
## HANDED OVER rather than resampled on the other side, because a second copy could drift from the
## one the pose lookup actually uses -- exactly the error the pdt check exists to catch.
signal frame_sampled(pdt: int, now_usec: int, xr_clock_offset_ns: int, pdt_stamping: bool, delta: float)
## Once, at the end of _setup_xr_locator, and only when this deploy has OpenXR at all. Late
## subscribers use get_head_locator() / get_xr_api(). to_world is _play_space_to_world as a
## Callable, so a consumer testing the locator tests the TIME argument and not a second copy of
## the conversion.
signal xr_locator_ready(locator: OpenXRHeadLocator, xr_api: OpenXRAPIExtension, to_world: Callable)

## Master switch. The camera pipeline is only started while true. Toggling it off at runtime stops
## dispatching new detections (in-flight ones finish) and pauses every published tracker (pose
## invalidated, state PAUSED), like a lost marker.
@export var enabled := true

# --- Marker configuration -------------------------------------------------------------------
# CALIBRATION OWNERSHIP, once for this whole block. The values below are native properties of the
# OpenCVProcessor (see src/OpenCVProcessor.h, which carries the provenance of every number). That
# processor is owned by composition and never enters the scene tree, so its inspector page is
# invisible -- these exports are the authored surface, and each one WRITES THROUGH to the property
# behind it.
# Both halves are needed and neither is redundant: PackedScene applies exported values BEFORE
# _ready, when `processor` is still null and the setter can do nothing, while inspector edits
# arrive AFTER _ready, when the setter is the only path. So _push_calibration() runs once right
# after OpenCVProcessor.new(), and the setters carry every change from then on -- including an edit
# in the remote inspector of a running deploy, which takes effect on the next frame.
# The defaults here are IDENTICAL to the C++ defaults on purpose, so the push is a no-op for an
# untouched node; project/tests/drop_in_test.gd asserts exactly that. Changing one without the
# other silently overrides a calibration for every consumer.

## Fallback physical side length, in meters, for every marker id WITHOUT a marker_sizes entry.
## Used as the solvePnP marker size (sets the pose's metric scale) AND published as the
## tracker's bounds_size, so the two can never disagree.
@export_range(0.01, 0.3, 0.001, "or_greater", "suffix:m") var default_marker_size := 0.1:
	set(value):
		default_marker_size = value
		if processor != null:
			processor.aruco_patch_size = value

## Ground-truth lookup table: index = marker id, value = physical side length in meters.
## 0 = unset -> that id falls back to default_marker_size (as do all ids >= the array length),
## so an untouched table behaves exactly like a single-size setup.
@export_range(0.01, 0.3, 0.001, "or_greater", "suffix:m") var marker_sizes: Array[float] = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]:
	set(value):
		marker_sizes = value
		if processor != null:
			processor.aruco_patch_sizes = PackedFloat64Array(value)

## Which predefined dictionary the detector searches. The printed markers decide this, not taste --
## and the two are not interchangeable, since ids collide across dictionaries. 36h12 is the default
## and the better code (36 bits, minimum Hamming distance 12, against 16 bits and distance 4);
## 4x4_50 is here because existing printed material uses it and paper cannot be recompiled.
## Values match OpenCVProcessor.MARKER_DICT_ARUCO_MIP_36H12 / _4X4_50 -- compare against those
## constants rather than against 0/1. Typed int with an explicit hint rather than the native enum
## type, so this file parses even against an older extension build that has neither.
@export_enum("ArUco MIP 36h12", "4x4 (50 ids)") var marker_dictionary: int = 0:
	set(value):
		marker_dictionary = value
		if processor != null:
			processor.marker_dictionary = value

## How long a marker may be absent from detection results before its tracker's pose is
## invalidated and its spatial_tracking_state goes to PAUSED. Detection is noisy (motion blur,
## glancing angles drop a marker for a frame or two); half a second bridges the dropouts.
@export_range(0, 5000, 10.0, "suffix:ms") var marker_lost_timeout_ms := 500.0

## How long after the last detection a PAUSED tracker is fully removed from the XRServer
## (state STOPPED first, mirroring the engine's removal path). 0 disables removal: trackers
## then stay registered (paused) for the whole session, bounded by the dictionary's id count.
@export_range(0, 120, 0.5, "suffix:s") var marker_stopped_timeout_s := 10.0

# --- Debugging ------------------------------------------------------------------------------
# Single switch for ALL debug output, this script's and both C++ classes'. Off by default.
# Errors are never gated and print regardless.
# Format for every debug line: "[opencv_aruco] [aruco_marker_tracking::function] event: k=v" --
# the fixed prefix is what makes them findable in logcat (adb logcat | grep opencv_aruco).
@export var debug_prints_enabled := false:
	set(value):
		debug_prints_enabled = value
		# Mirror every change into BOTH C++ statics right away. They gate their own log lines on a
		# flag only GDScript can set, and pushing once in _ready would mean a toggle from the
		# remote inspector of a running deploy silenced this script while the extension kept
		# logging. Assigning the property inside its own setter does not recurse in GDScript.
		OpenCVProcessor.set_debug_prints_enabled(value)
		OpenXRHeadLocator.set_debug_prints_enabled(value)

## Upload each pushed camera frame into a preview texture (see camera_feed_started). Off by
## default because it is a full 640x480 upload per frame on the Quest for something only a debug
## overlay looks at. The PULL path ignores this -- its CameraTexture exists either way.
@export var camera_preview_enabled := false

# --- Camera calibration ---------------------------------------------------------------------
## Intrinsics (fx, fy, cx, cy) in pixels for the NATIVE frame (640x480 on the Quest passthrough
## camera "50"). image_downscale_factor is applied to them at use time, so NEVER bake it in here.
## Rewritten at runtime off Android, when the camera frame size changes -- see
## _approximate_desktop_intrinsics.
@export var camera_intrinsics := Vector4(436.90348444, 436.86219469, 321.49573022, 239.71397166):
	set(value):
		camera_intrinsics = value
		if processor != null:
			processor.camera_intrinsics = value

## OpenCV distCoeffs (k1, k2, p1, p2, k3) for the Quest passthrough lens; an EMPTY array means
## "no distortion". From the SAME calibration run as camera_intrinsics and replaced with it in one
## edit: a focal length from one run beside distortion coefficients from another describes no lens
## that exists.
@export var camera_distortion: PackedFloat64Array = [-0.00323431, 0.02542156, -0.00016776, 0.00090852, -0.02965119]:
	set(value):
		camera_distortion = value
		if processor != null:
			processor.camera_distortion = value

## Detection resolution knob: 1.0 = native frame, 0.5 = half width AND half height -> markedly
## cheaper detection, at the price of small or distant markers dropping below the resolution the
## detector needs. The C++ side scales the intrinsics by the same (clamped) factor, so the two can
## never disagree.
@export_range(0.1, 1.0, 0.05) var image_downscale_factor := 1.0:
	set(value):
		image_downscale_factor = value
		if processor != null:
			processor.image_downscale_factor = value

# Horizontal FOV assumed by the desktop pinhole guess (see _approximate_desktop_intrinsics).
# 65deg is the middle of the usual laptop-webcam range. Hardcoded rather than exported on
# purpose: it is the single number in a deliberately rough fallback, and a knob here would
# invite tuning it by eye instead of running tools/cameraCalibration.py.
const DESKTOP_ASSUMED_HFOV_DEG := 65.0

## Physical passthrough-camera pose relative to the head/VIEW pose it gets combined with.
## MEASURED -- these are NOT the raw Camera2 metadata.
## The quaternion is ~168.8deg about X = the Android sensor->camera-optical 180deg X-flip PLUS
## the camera's real ~11deg pitch. The C++ marker pose already contains that same 180deg flip
## (its negate-Y/Z change of basis), so the C++ setter multiplies by Quaternion(1,0,0,0) (=180deg
## about X) to cancel the flip and keep ONLY the physical mounting tilt.
## The translation is in the sensor frame (X right, Y up, Z toward viewer), which matches Godot
## camera axes -> no sign flips.
##
## These two are ONE calibration and must be replaced as a PAIR.
##
## DO NOT "fix" them by pasting in what the Quest's ACAMERA_LENS_POSE_ROTATION / _TRANSLATION
## report at startup, however authoritative that dump looks. It is gyro-referenced
## (LENS_POSE_REFERENCE == GYROSCOPE): the metadata describes the camera relative to the IMU,
## while the head pose it is combined with HERE is the VIEW pose. Those two frames differ by the
## IMU's mounting rotation, which NEITHER api exposes, so the difference cannot be looked up, only
## measured. That raw dump is what this project shipped until 2026-08 and it costs ~0.9deg: a
## standing offset of ~15mm at 1m, which no pose-path change can touch.
##
## Provenance: solved by tools/handeye_solve.py from ~500 captured samples of marker 0
## (H_i * L * M_i collapsing to one world pose at 1.2mm median / 3.5mm p90), then verified in
## CAMERA PIXELS with the reprojection overlay. Re-measure with that tool rather than editing by
## eye -- and only against the same physical camera (the left Quest passthrough camera, id "50").
@export var lens_rotation_raw := Quaternion(-0.9951163, -0.0028897487, 0.0037281485, 0.098596975):
	set(value):
		lens_rotation_raw = value
		if processor != null:
			processor.lens_rotation_raw = value
@export var lens_translation := Vector3(-0.03352603, -0.017866991, -0.058882877):
	set(value):
		lens_translation = value
		if processor != null:
			processor.lens_translation = value

# --- Capture-time head pose -----------------------------------------------------------------
## How old a frame's pixels are when they reach us -- sensor -> ISP -> delivery is 1-2 camera
## frames. Pairing those old pixels with the LIVE head pose bakes an error proportional to head
## speed, so a fresh detection first "drags" with the head, then settles.
##
## ONLY the PULL path uses this: the CameraX push path carries a real sensor timestamp and needs
## no guess. Read it as the physical quantity -- pixel age -- and nothing else. It used to absorb
## a second, unrelated term on the pull path (OpenXR's 40-80ms prediction lead, which is why the
## pre-merge readback branch tuned this to 90ms while the CameraX branch measured 50ms); the
## locator removes that term wherever the runtime answers, so the number here is now just latency.
## Where the locator cannot answer, the history fallback still under-corrects by that lead --
## visible as a rising locate_fallbacks count in the flow trace.
## Tune on device: marker still drags WITH the head -> raise; marker lags behind -> lower.
@export_range(0, 300, 1.0, "or_greater", "suffix:ms") var camera_latency_ms := 50.0

## Residual trim of the pose lookup, in ms; positive = use an OLDER head pose. Expected to stay at
## 0 on the push path: the sensor timestamp removes the variable pipeline delay and pdt-stamping
## removes the prediction lead. What is left is only what neither can see -- whether the sensor
## timestamp marks the start or the middle of the exposure.
@export_range(-20, 20, 0.1, "or_greater", "or_less", "suffix:ms") var pose_lookup_trim_ms := 0.0

## Ask the OpenXR runtime for the head pose at the frame's capture time (xrLocateSpace) instead of
## interpolating the pose history. For a time in the PAST that is the runtime's fused, measured
## estimate rather than the forecast the history holds -- and the history's entries are forecasts
## for a display time 40-80ms out, so the predictor's error rode into every marker pose, worst
## during exactly the fast head motion where it shows. A frame the runtime will not answer falls
## back to the history by itself, so neither setting can strand the app.
@export var use_xr_locate_space := true

# --- Debug: step-by-step flow tracing (learning aid, costs performance) ----------------------
# Every DEBUG_FLOW_EVERY-th camera frame is "traced": each station of the pipeline prints one
# line tagged with that frame's id, so ONE frame can be followed end to end:
#   (1) arrival -> (2) timestamp -> (3) lookup target -> (4) head pose at capture -> (5) handoff
#   -> (6) worker -> (7) detection -> (8) applying result -> (9) what the correction was worth
#   -> (10) markers published
# Tracing by id (instead of printing everything) keeps the lines of one frame together even
# though stations run on two threads and ~200ms apart. A SUB-switch of debug_prints_enabled.
const DEBUG_FLOW := true
const DEBUG_FLOW_EVERY := 30       # 1 = every frame (very chatty); 30 = ~1 traced frame per second

# --- State ----------------------------------------------------------------------------------

var processor: OpenCVProcessor

# id -> OpenXRMarkerTracker, the ONE registry of published trackers. A tracker is created on a
# marker's first detection, paused after marker_lost_timeout_ms without one, removed after
# marker_stopped_timeout_s. Because an id is only ever added through _publish_marker (which
# checks this dictionary first) and only ever removed together with its XRServer registration,
# one id can never own two trackers. Main thread only -- detection tasks never touch it.
var _trackers: Dictionary = {}
# id -> Time.get_ticks_usec() of the last detection result that contained it. Drives the pause/
# remove timeouts AND the public marker_age_ms(): a consumer asking "how stale is my last good
# pose?" must still get an answer after the tracker was paused or removed.
var _marker_last_seen: Dictionary = {}
# id -> last known PLAY-SPACE pose. Deliberately NOT pruned alongside the trackers: a consumer
# holding its last good pose through a dropout needs the pose to outlive the tracker. Bounded by
# the dictionary's id count (50 for 4x4_50, 250 for 36h12), so it cannot grow without limit.
var _marker_poses: Dictionary = {}

# --- Camera: PULL backend (CameraServer) ----------------------------------------------------
# Godot has a native CameraServer (Camera2) backend on Android since 4.5, so CameraServerExtension
# is only needed on desktop (Windows). Keep this var UNTYPED and instantiate via ClassDB so the
# script still parses on platforms where that class isn't registered.
var _camera_extension
var _cam_texture: CameraTexture
# Frame size camera_intrinsics has already been reconciled with; ZERO = none yet. On Android it
# only records the size (the exported Quest calibration is the right one and is never touched);
# off Android it is the size the pinhole guess was computed for. Compared per dispatch rather
# than latched once, because a CameraTexture hands out a 4x4 PLACEHOLDER Image before the feed's
# first real frame -- a guess derived from that is nonsense, and latched it would stay nonsense
# for the whole session.
var _intrinsics_frame_size := Vector2i.ZERO

# --- Camera: PUSH backend (GodotAndroidCamera / CameraX) ------------------------------------
# Both UNTYPED and acquired through load(ANDROID_CAMERA_SCRIPT): see the class comment.
var _android_cam_script
var _android_cam
var _android_cam_started := false
var _cam_clock_offset_ns := 0          # camera timestamp clock ns - Godot ticks ns
var _cam_ts_realtime := false          # feed stamps on boottime ("realtime") vs CLOCK_MONOTONIC ("unknown")
# Second clock bridge, for XrTime (CLOCK_MONOTONIC ns on the Quest) -> Godot ticks. Kept separate
# from _cam_clock_offset_ns because that one follows the FEED's clock, which is boottime for a
# "realtime" feed and would be wrong for pdt. On the Quest passthrough path ("unknown" -> also
# monotonic) the two hold the same value, and that is what makes an error in the offset cancel
# out of the pose lookup entirely -- see _resample_clock_offsets and _process.
var _xr_clock_offset_ns := 0
var _xr_stamp_poses := false           # stamp history entries with pdt instead of "now" (Quest only)
var _xr_lead_usec := 0                 # last measured pdt - now, carried over if pdt is briefly 0
var _cam_frame_count := 0
var _cam_fallback_count := 0           # frames whose timestamp failed the plausibility guard
var _preview_texture: ImageTexture     # debug preview fed from pushed frames (pull path uses _cam_texture)

# --- OpenXR head locator --------------------------------------------------------------------
# Null when OpenXR is not initialised (desktop without a headset); every use is guarded.
var _xr_api: OpenXRAPIExtension        # access to xrWaitFrame's predicted display time (XrTime)
var _head_locator: OpenXRHeadLocator
var _locate_fallback_count := 0        # frames on which the locator had no valid pose
var _last_pose_source := "history"     # trace-only; set by _head_pose_at_capture

# [t_usec, play-space head Transform3D] pairs, newest last; main thread only.
var _head_pose_history: Array = []

# --- Detection worker -----------------------------------------------------------------------
# On the Quest the OpenCV detection costs ~40ms, which run synchronously would cap the whole app.
# We run ONLY the detection (detectMarkers + solvePnP) off the main thread, as one-shot
# WorkerThreadPool tasks -- Godot owns the threads, so there is no Thread/Mutex/Semaphore
# lifecycle to manage here. At most ONE task is in flight; get_image() and all XRServer/tracker
# writes stay on the main thread.
var _detect_task_id := -1              # WorkerThreadPool task id; -1 = no task in flight
# input slot (main thread ONLY, so no lock): frames arriving while a task runs wait here; only the
# newest frame is kept (frame drop). The head pose sampled at the frame's capture time travels
# with the frame.
var _pending_image: Image
var _pending_capture_usec := 0
var _pending_head_pose := Transform3D.IDENTITY
var _has_pending := false
var _pending_frame_id := 0
# output slots, written by the task; the main thread reads them only AFTER
# wait_for_task_completion(), which is the synchronization point (no lock needed)
var _result_markers: Dictionary = {}
# id -> PackedVector2Array of the 4 marker corners in the frame's own pixel space, straight
# from the C++ detector. Debug data for the reprojection overlay ONLY.
var _result_corners: Dictionary = {}
var _result_capture_usec := 0
var _result_head_pose := Transform3D.IDENTITY   # play-space head pose the markers were baked with
var _result_frame_id := 0
# The frame the in-flight (or just finished) task is working on, handed to detection_applied once
# the worker is done -- that detection's corners only exist then, so a frame emitted at arrival
# time could never carry them. Overwritten on every dispatch.
var _detecting_img: Image
var _flow_frame_counter := 0           # frame ids for the pull path (push path uses _cam_frame_count)


#######################################################################################################
# --- Public marker API -------------------------------------------------------------------------
# The standard consumption route is XRServer.tracker_added + XRAnchor3D (see the class comment).
# These id-keyed helpers exist on top for consumers (avatar rigs, debug gizmos) that want poses
# without going through tracker objects. Main thread only.

## The published tracker for a marker id, or null while none is registered (never seen, or
## removed after marker_stopped_timeout_s). The returned object IS the one the XRServer holds.
func get_marker_tracker(id: int) -> OpenXRMarkerTracker:
	return _trackers.get(id)

## The XRServer tracker name a marker id is (or would be) published under -- bind an
## XRAnchor3D's `tracker` property to this to follow a known marker id.
func get_tracker_name_for(id: int) -> StringName:
	return StringName(TRACKER_NAME_PREFIX + str(id))

## Last known PLAY-SPACE pose (the space XR trackers report in; an XRAnchor3D under the
## XROrigin3D lands exactly there). IDENTITY if never detected -- pair with has_marker() if
## that would be indistinguishable from a real pose for you.
func get_marker_pose(id: int) -> Transform3D:
	return _marker_poses.get(id, Transform3D.IDENTITY)

## Last known pose converted to WORLD space (applies world scale, the XR reference frame and the
## current world origin, i.e. the XROrigin3D's global transform). IDENTITY if never detected.
## NOTE the pairing: get_marker_pose() is PLAY space, this one is WORLD. Code ported from a
## pre-addon version of this pipeline, where get_marker_pose() returned world space, must move to
## THIS function -- the other one will compile, run, and be wrong by the XROrigin3D transform.
func get_marker_world_pose(id: int) -> Transform3D:
	if not _marker_poses.has(id):
		return Transform3D.IDENTITY
	return _play_space_to_world(_marker_poses[id])

## True once this marker has been detected at least once. It may be stale by now.
func has_marker(id: int) -> bool:
	return _marker_poses.has(id)

## Milliseconds since this marker was last detected; INF if never. INF compares correctly
## against any max_age below, so a never-seen id is simply never fresh.
func marker_age_ms(id: int) -> float:
	if not _marker_last_seen.has(id):
		return INF
	return (Time.get_ticks_usec() - _marker_last_seen[id]) / 1000.0

## True only if EVERY id is fresh -- a pose averaged over several markers is only as good as
## its weakest one. Defaults to the tracker pause grace period, so "the tracker still has
## tracking data" and "the consumer is tracking" stay the same statement.
func markers_fresh(ids: Array, max_age_ms := -1.0) -> bool:
	if max_age_ms < 0.0:
		max_age_ms = marker_lost_timeout_ms
	for id in ids:
		if marker_age_ms(id) > max_age_ms:
			return false
	return true

## True once every id has been seen at least once, stale or not. Use this to decide whether a
## consumer may be shown at all; markers_fresh() decides whether to move it.
func markers_ever_seen(ids: Array) -> bool:
	for id in ids:
		if not _marker_poses.has(id):
			return false
	return true

## Centroid + mean rotation of the given markers (play space). Unknown ids are skipped;
## IDENTITY if none are known.
##
## Sum-then-normalise is the cheap quaternion mean, and for exactly two markers it is identical
## to slerp(q0, q1, 0.5). slerp cannot generalise it: it takes only two, and chaining it is not
## associative, so the result would depend on marker order. Each quaternion is sign-aligned
## against the first KNOWN one, because q and -q are the same rotation and would otherwise
## cancel instead of average.
func get_average_marker_pose(ids: Array) -> Transform3D:
	var centre := Vector3.ZERO
	var acc := Quaternion(0, 0, 0, 0)
	var ref := Quaternion.IDENTITY
	var n := 0
	for id in ids:
		if not _marker_poses.has(id):
			continue
		var x: Transform3D = _marker_poses[id]
		var q := x.basis.get_rotation_quaternion()
		# Counting KNOWN ids rather than using the loop index is what makes skipping safe: the
		# reference is the first quaternion actually accumulated, not the first id asked for.
		if n == 0:
			ref = q
		elif ref.dot(q) < 0.0:
			q = -q
		centre += x.origin
		acc = Quaternion(acc.x + q.x, acc.y + q.y, acc.z + q.z, acc.w + q.w)
		n += 1
	if n == 0:
		return Transform3D.IDENTITY
	return Transform3D(Basis(acc.normalized()), centre / float(n))

## The physical side length in meters this id's poses were solved with -- the SAME number
## solvePnP used, resolved by the C++ side against marker_sizes/default_marker_size.
func get_marker_size(id: int) -> float:
	return processor.get_marker_size(id) if processor != null else default_marker_size

## The live camera texture once the feed is running, else null. A CameraTexture on the pull path,
## an ImageTexture on the push path (and null there unless camera_preview_enabled).
## camera_feed_started announces the moment it becomes available.
func get_camera_texture() -> Texture2D:
	return _cam_texture if _cam_texture != null else _preview_texture

# --- Escape hatches for the demo-only measurement nodes ---------------------------------------
# Documented so the apparatus can stay out of this addon; nothing in here depends on them.

## The OpenCVProcessor behind this node, for code that needs project_marker_corners() or
## get_lens_pose() -- the reprojection overlay and the hand-eye capture.
func get_processor() -> OpenCVProcessor:
	return processor

## The head locator, or null when this deploy has no OpenXR. See xr_locator_ready.
func get_head_locator() -> OpenXRHeadLocator:
	return _head_locator

## The OpenXRAPIExtension, or null when this deploy has no OpenXR. See xr_locator_ready.
func get_xr_api() -> OpenXRAPIExtension:
	return _xr_api

# --- Capability compat helpers ---------------------------------------------------------------
# Mirror OpenXRSpatialMarkerTrackingCapability's support queries for code that feature-checks
# before subscribing. Both selectable dictionaries are ArUco, hence ArUco only.

func is_aruco_supported() -> bool:
	return true

func is_qrcode_supported() -> bool:
	return false

func is_micro_qrcode_supported() -> bool:
	return false

func is_april_tag_supported() -> bool:
	return false

#######################################################################################################

# Hand every exported calibration value to the processor once, immediately after it is built.
# The setters above cannot do this on their own -- see the CALIBRATION OWNERSHIP block.
func _push_calibration() -> void:
	processor.camera_intrinsics = camera_intrinsics
	processor.camera_distortion = camera_distortion
	processor.image_downscale_factor = image_downscale_factor
	processor.lens_rotation_raw = lens_rotation_raw          # C++ setter rebuilds the lens pose
	processor.lens_translation = lens_translation
	processor.aruco_patch_size = default_marker_size
	processor.aruco_patch_sizes = PackedFloat64Array(marker_sizes)
	processor.marker_dictionary = marker_dictionary


# Put the current marker sizes on the live trackers' bounds_size. The id -> size resolution itself
# is get_marker_size(), a native method of the processor, so there is nothing left to mirror into
# a table for the C++ side -- it reads its own properties at the top of every detection.
# Trackers got their bounds_size at publish time; without this a size change would only reach
# consumers after the marker had been lost and re-published. Compared approximately because
# bounds_size round-trips through 32-bit floats.
# Main thread only. (The "no task in flight" contract went with the table it used to rebuild.)
func _sync_marker_sizes() -> void:
	for id in _trackers:
		var size := processor.get_marker_size(id)
		var tracker: OpenXRMarkerTracker = _trackers[id]
		if not is_equal_approx(tracker.bounds_size.x, size):
			tracker.bounds_size = Vector2(size, size)
			if debug_prints_enabled:
				print("[opencv_aruco] [aruco_marker_tracking::_sync_marker_sizes] bounds resized: id=%d size=%.3f" % [id, size])

#######################################################################################################

func _ready() -> void:
	# The property setter already pushed this into the extension at scene-instantiation time;
	# repeat it here so the flag is also correct when the scene does NOT override the default
	# (the setter never fires then) and a previous run left the static true. Still BEFORE new().
	OpenCVProcessor.set_debug_prints_enabled(debug_prints_enabled)
	OpenXRHeadLocator.set_debug_prints_enabled(debug_prints_enabled)
	processor = OpenCVProcessor.new()
	_push_calibration()
	# Everything that prints or touches the device, deferred out of the C++ constructor so the
	# flag above can gate it. Nothing in the detection depends on it.
	processor.dump_build_info_and_intrinsics()

	# If Godot's own OpenXR marker tracking is also switched on, both backends would publish
	# TRACKER_ANCHOR trackers side by side. That is allowed (names cannot collide) but almost
	# never intended -- say so instead of silently double-tracking.
	if ProjectSettings.get_setting("xr/openxr/extensions/spatial_entity/enabled", false) \
			and ProjectSettings.get_setting("xr/openxr/extensions/spatial_entity/enable_marker_tracking", false) \
			and ProjectSettings.get_setting("xr/openxr/extensions/spatial_entity/enable_builtin_marker_tracking", false):
		push_warning("[opencv_aruco] Godot's built-in OpenXR marker tracking is enabled in the " +
				"project settings alongside ArucoMarkerTracking; both will publish anchor trackers.")

	if not enabled:
		return

	# BEFORE the camera branch below, which can start delivering frames synchronously: those
	# frames want the locator. Safe here because OpenXR is initialised by the ENGINE, before the
	# main scene is loaded at all (xr/openxr/enabled=true in project.godot), so the
	# is_initialized() check does not depend on node order.
	_setup_xr_locator()

	# PUSH if the CameraX plugin is present, PULL otherwise. _setup_android_camera reports whether
	# it could actually attach, so a half-installed plugin falls through instead of leaving the
	# node with no camera at all.
	if OS.get_name() == "Android" and Engine.has_singleton("GodotAndroidCamera") and _setup_android_camera():
		return
	_setup_camera_server()


# Build the OpenXR head locator, or leave it null when there is no OpenXR at all (desktop run
# without a headset) -- every caller checks, and _xr_api is tied to the same condition so nothing
# below ever pokes a dead OpenXRAPI singleton once per frame.
# The VIEW space the C++ side creates is a CHILD of the XR session: the runtime destroys it with
# the session, so it has to be given up on session_stopping. The node's destructor is too late --
# by scene teardown the session is usually already gone.
func _setup_xr_locator() -> void:
	var xr_interface: XRInterface = XRServer.find_interface("OpenXR")
	if xr_interface == null or not xr_interface.is_initialized():
		if debug_prints_enabled:
			print("[opencv_aruco] [aruco_marker_tracking::_setup_xr_locator] no OpenXR: head locator disabled, pose history stays the only path")
		return
	_xr_api = OpenXRAPIExtension.new()
	_head_locator = OpenXRHeadLocator.new()
	add_child(_head_locator)
	# connect() by NAME, not xr_interface.session_stopping.connect(): the var is statically typed
	# XRInterface, which has no such signal -- only the OpenXRInterface behind it does.
	if xr_interface.has_signal("session_stopping"):
		xr_interface.connect("session_stopping", Callable(_head_locator, "release"))
	xr_locator_ready.emit(_head_locator, _xr_api, _play_space_to_world)
	if debug_prints_enabled:
		print("[opencv_aruco] [aruco_marker_tracking::_setup_xr_locator] head locator created: use_xr_locate_space=%s" % use_xr_locate_space)


# --- PULL backend: Godot CameraServer --------------------------------------------------------

func _setup_camera_server() -> void:
	if OS.get_name() == "Android":
		# Quest without the CameraX plugin: request camera access; the native CameraServer
		# surfaces feeds once granted.
		OS.request_permission("android.permission.CAMERA")
		OS.request_permission("horizonos.permission.HEADSET_CAMERA")
	elif ClassDB.class_exists("CameraServerExtension"):
		# Desktop (Windows): custom backend that registers the webcam as a feed. Reached through
		# ClassDB because it is a NATIVE class that may be absent -- which is exactly why the same
		# trick does not work for the CameraX plugin (a GDScript global class; see the class
		# comment).
		_camera_extension = ClassDB.instantiate("CameraServerExtension")  # keep reference alive

	# Since Godot 4.5, monitoring_feeds must be true before feeds are enumerated.
	CameraServer.monitoring_feeds = true
	CameraServer.camera_feeds_updated.connect(_on_camera_feeds_updated)
	_on_camera_feeds_updated()                          # in case a feed is already present


func _on_camera_feeds_updated() -> void:
	if _cam_texture != null:
		return                                          # already initialised
	var feed_count := CameraServer.get_feed_count()
	if feed_count == 0:
		return

	# log every available feed so we can see which index is the (passthrough) camera on Quest
	if debug_prints_enabled:
		for i in range(feed_count):
			var f := CameraServer.get_feed(i)
			print("[opencv_aruco] [aruco_marker_tracking::_on_camera_feeds_updated] feed: index=%d id=%d name=%s" % [i, f.get_id(), f.get_name()])

	# Quest exposes 3 feeds: "1 | FRONT" plus the passthrough pair "50 | BACK" / "51 | BACK".
	# The world-facing ("BACK") cameras are the passthrough ones we want; feed 0 (FRONT) is the
	# wrong camera. Desktop has a single feed, so it falls through to 0.
	var feed: CameraFeed = null
	for i in range(feed_count):
		var f := CameraServer.get_feed(i)
		if "BACK" in f.get_name():
			feed = f
			break
	if feed == null:
		feed = CameraServer.get_feed(0)

	# Format MUST be chosen before activating, else "format index -1" and no frames.
	var formats := feed.get_formats()
	if debug_prints_enabled:
		for j in range(formats.size()):
			print("[opencv_aruco] [aruco_marker_tracking::_on_camera_feeds_updated] format: index=%d value=%s" % [j, formats[j]])
	if formats.size() > 2:
		feed.set_format(2, {})        # feed format:10 1280x1280 YUV_420_888, feed format:2 640x480
	elif formats.size() > 0:
		feed.set_format(0, {})

	feed.set_active(true)                               # start delivering frames
	_cam_texture = CameraTexture.new()
	_cam_texture.camera_feed_id = feed.get_id()
	_cam_texture.which_feed = CameraServer.FEED_RGBA_IMAGE
	if debug_prints_enabled:
		print("[opencv_aruco] [aruco_marker_tracking::_on_camera_feeds_updated] feed activated: id=%d name=%s feed_count=%d" % [feed.get_id(), feed.get_name(), feed_count])
	camera_feed_started.emit(_cam_texture)


# --- PUSH backend: GodotAndroidCamera (CameraX) ----------------------------------------------

# Returns false if the plugin's GDScript wrapper is not in the project, so the caller can fall
# back to the CameraServer. Everything here goes through untyped vars: see the class comment.
func _setup_android_camera() -> bool:
	if not ResourceLoader.exists(ANDROID_CAMERA_SCRIPT):
		push_warning("[opencv_aruco] The GodotAndroidCamera singleton is in this build but %s is not; " % ANDROID_CAMERA_SCRIPT
				+ "falling back to the CameraServer, which has no sensor timestamps.")
		return false
	_android_cam_script = load(ANDROID_CAMERA_SCRIPT)
	if _android_cam_script == null:
		return false
	_android_cam = _android_cam_script.new()
	add_child(_android_cam)
	_android_cam.camera_frame.connect(_on_android_camera_frame)
	# Start once BOTH permissions (android CAMERA + horizonos HEADSET_CAMERA) are granted; the
	# result signal fires once per permission, so re-check on every grant.
	get_tree().on_request_permissions_result.connect(_on_permission_result)
	if _android_cam.request_camera_permissions():
		_start_android_camera()
	return true


func _on_permission_result(_permission: String, granted: bool) -> void:
	if granted and not _android_cam_started and _android_cam.request_camera_permissions():
		_start_android_camera()


func _start_android_camera() -> void:
	_android_cam_started = true
	# Pick the LEFT passthrough camera ("50"): the calibration exported above belongs to it.
	# Fall back to any world-facing feed on non-Quest devices.
	var cam_id := ""
	var cameras: Dictionary = _android_cam.get_available_cameras()
	for id in cameras:
		var info: Dictionary = cameras[id]
		if info.get("source", "") == "passthrough" and info.get("position", "") == "left":
			cam_id = id
			break
	if cam_id == "":
		for id in cameras:
			if cameras[id].get("facing", "") == "back":
				cam_id = id
				break
	# Map camera timestamps onto Time.get_ticks_usec(), the clock the head-pose history is
	# stamped with. WHICH camera clock to calibrate against depends on the feed:
	# "realtime" = boottime (elapsedRealtimeNanos); "unknown" (Quest passthrough) = typically
	# CLOCK_MONOTONIC, the very clock Godot's ticks run on. Using the boottime offset for a
	# monotonic feed is off by the headset's accumulated doze time since boot -- a silent,
	# run-dependent bias that made markers lag behind head motion.
	_cam_ts_realtime = str(cameras.get(cam_id, {}).get("timestamp_source", "unknown")) == "realtime"
	_resample_clock_offsets()
	# From here on the head-pose history is stamped with xrWaitFrame's predicted display time
	# rather than "now" (see _process). _xr_api is built in _setup_xr_locator, which runs first
	# and is the single point that decides whether this deploy has OpenXR at all; a frame where
	# the runtime reports no pdt falls back gracefully.
	_xr_stamp_poses = _xr_api != null
	if debug_prints_enabled:
		print("[opencv_aruco] [aruco_marker_tracking::_start_android_camera] starting camera: id=%s clock=%s feeds=%s" % [
				cam_id, "realtime" if _cam_ts_realtime else "monotonic", cameras])
		print("[opencv_aruco] [aruco_marker_tracking::_start_android_camera] flow setup: clock bridge calibrated, camera clock is ahead of Time.get_ticks_usec() by offset_s=%.3f offset_ns=%d" % [
				_cam_clock_offset_ns / 1.0e9, _cam_clock_offset_ns])
	# 640x480 matches the exported intrinsics; see ANDROID_CAM_FORMAT_LUMA for the format.
	_android_cam.start_camera(640, 480, false, cam_id, 0, 0, ANDROID_CAM_FORMAT_LUMA)


# Both clock bridges in ONE place so they can never drift apart. That matters: the pose lookup
# compares a camera timestamp mapped with _cam_clock_offset_ns against a history entry mapped with
# _xr_clock_offset_ns, so an error COMMON to both cancels out, while a divergence between them
# becomes a silent bias. On the Quest passthrough feed ("unknown" -> monotonic) they are literally
# the same number; only a "realtime" feed splits them, and then each is individually correct.
func _resample_clock_offsets() -> void:
	_xr_clock_offset_ns = _android_cam.get_monotonic_clock_offset_nanos()
	_cam_clock_offset_ns = _android_cam.get_clock_offset_nanos() if _cam_ts_realtime \
			else _xr_clock_offset_ns


# Runs on the main thread (plugin signals are marshalled onto the engine loop). data is the
# tight-packed Y plane; timestamp_ns is the sensor timestamp (start of exposure) of THIS frame.
func _on_android_camera_frame(timestamp_ns: int, data: PackedByteArray, width: int, height: int) -> void:
	# The `enabled` kill switch works by an early return in _process on the pull path -- which
	# frames arriving HERE bypass entirely, so it has to be repeated. Without this, detections
	# would keep being dispatched while _process is busy pausing every tracker.
	if not enabled:
		_pending_image = null
		_has_pending = false
		return

	# Sensor timestamp -> Godot clock. If the result is implausible (unexpected clock base, or
	# the offset went stale across a headset doze), resample the offset once and fall back to
	# the fixed-latency guess if it still disagrees.
	var now_usec := Time.get_ticks_usec()
	var cap_usec := (timestamp_ns - _cam_clock_offset_ns) / 1000
	var used_fallback := false
	if cap_usec > now_usec or now_usec - cap_usec > 500_000:
		_resample_clock_offsets()
		cap_usec = (timestamp_ns - _cam_clock_offset_ns) / 1000
		if cap_usec > now_usec or now_usec - cap_usec > 500_000:
			_cam_fallback_count += 1
			used_fallback = true
			cap_usec = now_usec - int(camera_latency_ms * 1000.0)
	var lag_ms := (now_usec - cap_usec) / 1000.0

	_cam_frame_count += 1
	var frame_id := _cam_frame_count
	var traced := _tracing(frame_id)
	if traced:
		# (1) what the camera handed us, (2) where that lands on Godot's clock.
		print("[opencv_aruco] [aruco_marker_tracking::_on_android_camera_frame] flow #%d (1) frame arrives: width=%d height=%d bytes=%d timestamp_ns=%d (grayscale Y-plane, 1 byte/pixel)" % [
				frame_id, width, height, data.size(), timestamp_ns])
		print("[opencv_aruco] [aruco_marker_tracking::_on_android_camera_frame] flow #%d (2) exposure time on godot clock: cap_usec=%d age_ms=%.1f source=%s" % [
				frame_id, cap_usec, lag_ms, "FALLBACK_GUESS_timestamp_rejected" if used_fallback else "sensor_timestamp"])

	# The lookup target is the exposure time itself: the history entries carry the time their pose
	# actually describes, so the two timelines already line up and only the residual trim is left.
	# cap_usec itself is NOT shifted -- it travels on as the frame's true age.
	var lookup_usec := cap_usec - int(pose_lookup_trim_ms * 1000.0)
	# Sensor timestamp -> XrTime for the locator. Both clock offsets are resampled together, so on
	# the Quest passthrough feed -- monotonic, the same clock XrTime runs on -- the two cancel and
	# timestamp_ns goes in untouched; only a "realtime" feed needs the detour through Godot's clock.
	var xr_time_ns := timestamp_ns - _cam_clock_offset_ns + _xr_clock_offset_ns \
			- int(pose_lookup_trim_ms * 1.0e6)
	if traced:
		print("[opencv_aruco] [aruco_marker_tracking::_on_android_camera_frame] flow #%d (3) pose lookup target: target_usec=%d trim_ms=%.1f (history is pdt-stamped, so no prediction lead to undo)" % [
				frame_id, lookup_usec, pose_lookup_trim_ms])

	var head_pose := _head_pose_at_capture(lookup_usec, xr_time_ns)
	if traced:
		print("[opencv_aruco] [aruco_marker_tracking::_on_android_camera_frame] flow #%d (4) head pose at capture: pos=%v source=%s locate_fallbacks=%d (play space, travels with the frame)" % [
				frame_id, head_pose.origin, _last_pose_source, _locate_fallback_count])

	if debug_prints_enabled and (_cam_frame_count == 1 or _cam_frame_count % 300 == 0):
		print("[opencv_aruco] [aruco_marker_tracking::_on_android_camera_frame] frame: count=%d width=%d height=%d lag_ms=%.1f clock=%s fallbacks=%d" % [
				_cam_frame_count, width, height, lag_ms,
				"realtime" if _cam_ts_realtime else "monotonic", _cam_fallback_count])

	var img := Image.create_from_data(width, height, false, Image.FORMAT_L8, data)

	# Debug preview, off by default: this is a full-frame GPU upload per camera frame for
	# something only an overlay looks at.
	if camera_preview_enabled:
		if _preview_texture == null:
			_preview_texture = ImageTexture.create_from_image(img)
			camera_feed_started.emit(_preview_texture)
		else:
			_preview_texture.update(img)

	_dispatch_detection(img, cap_usec, head_pose, frame_id)


# True if this frame's journey should be traced. The SAME id gives the SAME answer at every
# station, so all prints belonging to one frame appear together (see DEBUG_FLOW).
func _tracing(frame_id: int) -> bool:
	return debug_prints_enabled and DEBUG_FLOW and frame_id % DEBUG_FLOW_EVERY == 0

####################################################################################################

func _process(delta: float) -> void:
	# (a) Head-pose history: one [t_usec, play-space pose] sample per rendered frame, so a finished
	# detection can be baked with the pose the head actually had at the frame's capture time.
	# Sampled EVERY render frame, before any early return below: _head_pose_at interpolates
	# between the two nearest samples, so its accuracy is bounded by the sampling period.
	#
	# Each sample is stamped with the time its pose DESCRIBES, not the time it was read. OpenXR
	# hands out poses PREDICTED for xrWaitFrame's display time, so stamping them with "now" was a
	# per-sample lie that a single constant could only ever cancel on average. Measured on the
	# Quest that lead is 40-80ms and not constant in either direction: it shrinks as the render
	# rate rises and sawtooths ~40ms peak-to-peak within that, because pdt steps in whole 13.89ms
	# display quanta while the wall clock runs on. Giving every entry its own pdt cancels all of
	# it. Off the push path _xr_stamp_poses stays false and this degrades to "now"-stamping.
	var now_usec := Time.get_ticks_usec()
	var pdt := _xr_api.get_predicted_display_time() if _xr_api != null else 0
	var pose_usec := now_usec + _xr_lead_usec    # pull path: lead stays 0 -> stamped with "now"
	if _xr_stamp_poses and pdt != 0:
		pose_usec = (pdt - _xr_clock_offset_ns) / 1000
		_xr_lead_usec = pose_usec - now_usec
	_head_pose_history.append([pose_usec, _head_pose_now()])
	# Pruned against the NEWEST stamp rather than now_usec: the entries sit in the future once
	# they are pdt-stamped, and measuring the window from "now" would silently shorten it.
	while _head_pose_history.size() > 1 and _head_pose_history[0][0] < pose_usec - 500_000:
		_head_pose_history.pop_front()

	# (b) Everything the demo-only timing checks need, handed over rather than read back.
	frame_sampled.emit(pdt, now_usec, _xr_clock_offset_ns,
			_xr_stamp_poses and not _cam_ts_realtime, delta)

	# (c) Collect the latest finished detection and publish it (main thread -> XRServer and
	# tracker writes are safe here).
	_poll_detection_task()

	if not enabled:
		# With dispatching off no detection result will ever pause these through the prune
		# pass, so do it here: a published tracker must not claim TRACKING (with an aging
		# pose) while the pipeline is switched off. State-guarded, so this is a no-op after
		# the first disabled frame.
		for id in _trackers:
			var tracker: OpenXRMarkerTracker = _trackers[id]
			if tracker.spatial_tracking_state == OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_TRACKING:
				tracker.invalidate_pose(&"default")
				tracker.spatial_tracking_state = OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_PAUSED
		return

	# (d) PULL path only: read the newest CameraServer frame. On the push path _cam_texture stays
	# null and frames arrive through _on_android_camera_frame instead.
	if _cam_texture == null:
		return

	# Readback ONLY when no detection is running. Detection costs ~40ms while _process runs at the
	# render rate, so an unconditional get_image() paid the full GPU->CPU stall several times per
	# detection and threw all but the last one away. Unlike the push path there is no frame to
	# park: declining to pull IS the frame drop.
	if _detect_task_id != -1:
		return

	var readback_t0 := Time.get_ticks_usec()
	var img := _cam_texture.get_image()
	if img == null:
		return
	# format lookup table https://docs.godotengine.org/en/stable/classes/class_image.html#enum-image-format
	if debug_prints_enabled:
		print("[opencv_aruco] [aruco_marker_tracking::_process] readback_ms=%.2f image_format=%d" % [(Time.get_ticks_usec() - readback_t0) / 1000.0, img.get_format()])

	# The exported calibration belongs to the Quest passthrough lens; on any other camera it is
	# simply wrong, so derive a pinhole guess from the frame we just read back.
	# The `_detect_task_id == -1` half of the condition is redundant TODAY (the guard above
	# already returned) and is written out anyway: it is the caller contract, and the early
	# return that currently implies it is a performance optimisation that someone could move.
	if _detect_task_id == -1 and _intrinsics_frame_size != Vector2i(img.get_width(), img.get_height()):
		_approximate_desktop_intrinsics(img)

	# No sensor timestamp on this path -- approximate the capture time as camera_latency_ms ago.
	# The locator can still be used, and better: pdt IS an XrTime, so shifting it by the same
	# latency needs no clock bridge at all (which is just as well, since _xr_clock_offset_ns is
	# only ever calibrated by the CameraX plugin).
	_flow_frame_counter += 1
	var capture_usec := now_usec - int(camera_latency_ms * 1000.0)
	var lookup_usec := capture_usec - int(pose_lookup_trim_ms * 1000.0)
	var xr_time_ns := 0
	if pdt != 0:
		xr_time_ns = pdt - int((camera_latency_ms + pose_lookup_trim_ms) * 1.0e6)
	if _tracing(_flow_frame_counter):
		print("[opencv_aruco] [aruco_marker_tracking::_process] flow #%d (1-3) pull path: no sensor timestamp, capture time guessed as now - camera_latency_ms=%.1f" % [
				_flow_frame_counter, camera_latency_ms])
	var head_pose := _head_pose_at_capture(lookup_usec, xr_time_ns)
	if _tracing(_flow_frame_counter):
		print("[opencv_aruco] [aruco_marker_tracking::_process] flow #%d (4) head pose at capture: pos=%v source=%s locate_fallbacks=%d" % [
				_flow_frame_counter, head_pose.origin, _last_pose_source, _locate_fallback_count])
	_dispatch_detection(img, capture_usec, head_pose, _flow_frame_counter)


# Hand a frame + its capture time + the head pose at that time to the detection task; at most
# one task runs at a time, and only the newest frame waits for the slot (frame drop).
func _dispatch_detection(img: Image, capture_usec: int, head_pose: Transform3D, frame_id: int) -> void:
	_poll_detection_task()               # a just-finished task frees the slot for this frame
	if _detect_task_id != -1:
		# Task still running -> park the frame; overwrites any unconsumed frame -> frame drop.
		var was_pending := _has_pending
		var dropped_id := _pending_frame_id
		_pending_image = img
		_pending_capture_usec = capture_usec
		_pending_head_pose = head_pose
		_pending_frame_id = frame_id
		_has_pending = true
		if _tracing(frame_id):
			print("[opencv_aruco] [aruco_marker_tracking::_dispatch_detection] flow #%d (5) task busy, frame parked: %s" % [
					frame_id,
					("dropped_frame=%d (newest frame wins)" % dropped_id) if was_pending else "pending_slot_was_free=true"])
		return
	_start_detection_task(img, capture_usec, head_pose, frame_id)
	if _tracing(frame_id):
		print("[opencv_aruco] [aruco_marker_tracking::_dispatch_detection] flow #%d (5) handed to worker: task_id=%d" % [
				frame_id, _detect_task_id])


# Main thread only. If the in-flight task has finished: clean it up, apply its result, and start
# the pending frame (if any). Called from _process AND before every dispatch, so a finished task
# is collected at render rate or camera rate, whichever fires first.
# NOTE this means tracker publication and markers_updated can fire from the camera callback and
# not only during _process. Still the main thread -- plugin signals are marshalled onto the engine
# loop -- but a consumer must not assume the two are ordered against its own _process.
func _poll_detection_task() -> void:
	if _detect_task_id == -1 or not WorkerThreadPool.is_task_completed(_detect_task_id):
		return
	# Mandatory cleanup of every finished task; returns immediately here (the task is done) and
	# doubles as the memory barrier that makes the task's _result_* writes visible to this thread.
	WorkerThreadPool.wait_for_task_completion(_detect_task_id)
	_detect_task_id = -1
	_apply_detection_result()
	# Only now do the frame and its corners both exist, so this is the earliest point at which the
	# overlay can be emitted as one consistent pair -- and it has to happen BEFORE the pending
	# frame below is handed on, since that overwrites _detecting_img. Passing the image as a signal
	# ARGUMENT is what makes that ordering safe rather than merely documented.
	detection_applied.emit(_detecting_img, _result_corners, _result_markers, _result_head_pose)
	if _has_pending:
		var img := _pending_image
		var capture_usec := _pending_capture_usec
		var head_pose := _pending_head_pose
		var frame_id := _pending_frame_id
		_pending_image = null
		_has_pending = false
		_start_detection_task(img, capture_usec, head_pose, frame_id)
		if _tracing(frame_id):
			print("[opencv_aruco] [aruco_marker_tracking::_poll_detection_task] flow #%d (5b) pending frame handed to worker: task_id=%d" % [
					frame_id, _detect_task_id])


func _start_detection_task(img: Image, capture_usec: int, head_pose: Transform3D, frame_id: int) -> void:
	# Once per task is the right cadence for pushing size changes onto the published bounds, and
	# both entry points -- a fresh frame from _dispatch_detection and a parked one from
	# _poll_detection_task -- come through here.
	_sync_marker_sizes()
	# Held so _poll_detection_task can emit THIS frame once the task below has produced the
	# corners that belong to it.
	_detecting_img = img
	_detect_task_id = WorkerThreadPool.add_task(_detect_frame.bind(img, capture_usec, head_pose, frame_id),
			false, "opencv_aruco marker detection")


# Create-or-update the tracker for one detected marker (main thread only). The _trackers
# lookup is what guarantees one tracker per id. Field order mirrors the engine's
# _process_snapshot: everything is populated BEFORE add_tracker, so a tracker_added handler
# always sees a complete tracker.
func _publish_marker(id: int, pose: Transform3D) -> void:
	var tracker: OpenXRMarkerTracker = _trackers.get(id)
	var is_new := tracker == null
	if is_new:
		tracker = OpenXRMarkerTracker.new()
		# The OpenXRSpatialEntityTracker constructor already set type = TRACKER_ANCHOR.
		# Deliberately NOT set_entity(): we have no XrSpatialEntityIdEXT, and set_entity with an
		# invalid RID would rename the tracker to ".../null". See the class comment for the
		# naming contract.
		tracker.set_tracker_name(get_tracker_name_for(id))
		tracker.set_tracker_desc("ArUco marker %d (opencv_aruco)" % id)
		tracker.set_marker_type(OpenXRSpatialComponentMarkerList.MARKER_TYPE_ARUCO)
		tracker.set_marker_id(id)
		# marker_data stays unset: like the runtime route, ArUco markers carry their identity
		# in marker_id (marker_data is QR-code payload territory).
		_trackers[id] = tracker
	# Straight from the C++ side, so bounds_size and the size solvePnP used are the same number
	# by construction rather than by two matching copies of one rule.
	var size := processor.get_marker_size(id)
	tracker.bounds_size = Vector2(size, size)
	# Zero velocities and default HIGH confidence -- exactly what the engine's marker capability
	# passes. The pose is play-space; XRNode3D applies world scale + reference frame on top.
	tracker.set_pose(&"default", pose, Vector3.ZERO, Vector3.ZERO, XRPose.XR_TRACKING_CONFIDENCE_HIGH)
	tracker.spatial_tracking_state = OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_TRACKING
	if is_new:
		XRServer.add_tracker(tracker)
		if debug_prints_enabled:
			print("[opencv_aruco] [aruco_marker_tracking::_publish_marker] tracker added: id=%d name=%s trackers=%d" % [
					id, tracker.name, _trackers.size()])


# Apply the finished detection (main thread only). The markers come back from the C++ side
# already in PLAY space -- baked with the play-space head pose at the frame's capture time, which
# travelled with the frame -- so publishing is a plain assignment.
func _apply_detection_result() -> void:
	var markers: Dictionary = _result_markers
	var result_id: int = _result_frame_id
	var now_usec := Time.get_ticks_usec()
	if _tracing(result_id):
		print("[opencv_aruco] [aruco_marker_tracking::_apply_detection_result] flow #%d (8) applying result: capture_usec=%d result_age_ms=%.1f markers=%d" % [
				result_id, _result_capture_usec,
				(now_usec - _result_capture_usec) / 1000.0, markers.size()])
		# (9) what the whole timing correction was worth: the head pose the markers were baked
		# with vs. the LIVE one -- that difference is exactly the swim we avoid.
		var live_xform := _head_pose_now()
		var drift_cm := live_xform.origin.distance_to(_result_head_pose.origin) * 100.0
		var turn_deg := rad_to_deg(_result_head_pose.basis.get_rotation_quaternion().angle_to(
				live_xform.basis.get_rotation_quaternion()))
		print("[opencv_aruco] [aruco_marker_tracking::_apply_detection_result] flow #%d (9) head pose at capture: pos=%v live_pos=%v drift_cm=%.1f turn_deg=%.1f" % [
				result_id, _result_head_pose.origin, live_xform.origin, drift_cm, turn_deg])

	var seen_ids: Array = []
	for id in markers:
		_marker_poses[id] = markers[id]        # the id-keyed record the public API serves
		_marker_last_seen[id] = now_usec
		_publish_marker(id, markers[id])
		seen_ids.append(id)
		if _tracing(result_id):
			print("[opencv_aruco] [aruco_marker_tracking::_apply_detection_result] flow #%d (10) marker published: id=%d play_pos=%v" % [
					result_id, id, markers[id].origin])

	# Pause trackers whose marker has been missing for longer than the grace period, and remove
	# ones missing far longer. This runs HERE, on a fresh detection result, not on a timer:
	# absence is only evidence that a marker is gone once a frame that could have contained it
	# has been looked at. If the camera stalls, the trackers stay put instead of evaporating on
	# "no news". Both transitions mirror the engine's own marker capability: pause =
	# invalidate_pose + state PAUSED with the tracker kept (consumers hold the last pose and
	# XRAnchor3D.show_when_tracked hides); remove = state STOPPED first, "just in case there
	# are still references out there", then remove_tracker.
	# Iterating over keys() takes a copy, so erasing inside the loop is safe.
	for id in _trackers.keys():
		var tracker: OpenXRMarkerTracker = _trackers[id]
		var unseen_usec: int = now_usec - int(_marker_last_seen.get(id, now_usec))
		if unseen_usec <= int(marker_lost_timeout_ms * 1000.0):
			continue
		if marker_stopped_timeout_s > 0.0 and unseen_usec > int(marker_stopped_timeout_s * 1_000_000.0):
			tracker.invalidate_pose(&"default")
			tracker.spatial_tracking_state = OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_STOPPED
			XRServer.remove_tracker(tracker)
			_trackers.erase(id)
			# _marker_last_seen and _marker_poses are NOT erased with the tracker: they are the
			# public freshness/pose record behind marker_age_ms()/get_marker_pose(), and a
			# consumer asking "how stale is my last good pose?" must still get an answer after
			# the tracker is gone. Safe because this loop keys off _trackers.keys(), so a
			# leftover entry cannot resurrect a tracker, and _publish_marker rebuilds one
			# cleanly if the marker comes back.
			if debug_prints_enabled:
				print("[opencv_aruco] [aruco_marker_tracking::_apply_detection_result] tracker removed: id=%d unseen_ms=%.0f trackers=%d" % [
						id, unseen_usec / 1000.0, _trackers.size()])
		elif tracker.spatial_tracking_state == OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_TRACKING:
			# State guard: invalidate_pose once per dropout, not once per detection frame.
			tracker.invalidate_pose(&"default")
			tracker.spatial_tracking_state = OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_PAUSED
			if debug_prints_enabled:
				print("[opencv_aruco] [aruco_marker_tracking::_apply_detection_result] tracker paused: id=%d unseen_ms=%.0f" % [
						id, unseen_usec / 1000.0])

	# After the prune, so a handler reacting to this signal sees the final tracker state.
	if not seen_ids.is_empty():
		markers_updated.emit(seen_ids)


# Runs on a WorkerThreadPool thread: ONE frame's OpenCV detection (detectMarkers + solvePnP)
# off the main thread. Touches only `processor`, read-only config and the _result_* slots --
# never the scene tree or XRServer. Writing them without a lock is safe: the main thread reads
# the slots only after wait_for_task_completion() on this task.
func _detect_frame(img: Image, capture_usec: int, head_pose: Transform3D, frame_id: int) -> void:
	# No conversion: the C++ side handles 1ch (Quest Y-plane), 3ch (RGB), and 4ch (RGBA).
	var t0 := Time.get_ticks_usec()
	var traced := _tracing(frame_id)
	if traced:
		print("[opencv_aruco] [aruco_marker_tracking::_detect_frame] flow #%d (6) worker picked it up: frame_age_ms=%.1f" % [
				frame_id, (t0 - capture_usec) / 1000.0])
	# The head pose at CAPTURE time is the only thing that has to travel with the frame;
	# intrinsics, distortion, downscale, marker sizes, dictionary and the lens pose are all
	# properties of the processor, re-read there. Because the pose we hand in is PLAY space, the
	# markers come back in play space -- the C++ multiplies head_pose * lens_pose and is otherwise
	# indifferent to which space that was.
	# Out-parameter for the debug overlay: Dictionaries are shared references in Godot, so the C++
	# side writes the detected pixel corners into THIS instance. Built fresh per detection (rather
	# than clearing _result_corners) so the main thread can never see a half-filled dictionary --
	# the slot is only re-pointed at the end, past the same barrier as _result_markers.
	var corners: Dictionary = {}
	var markers: Dictionary = processor.detect_markers(img, head_pose, corners)
	# Guarded inline rather than via a helper function: a helper would build this string on
	# every detection before it could check the flag.
	if debug_prints_enabled:
		var detect_ms := (Time.get_ticks_usec() - t0) / 1000.0
		var tracking_fps := 1000.0 / detect_ms if detect_ms > 0.0 else 0.0
		print("[opencv_aruco] [aruco_marker_tracking::_detect_frame] detect_ms=%.1f tracking_fps=%.1f render_fps=%d frame_age_ms=%.1f markers=%d" % [
				detect_ms, tracking_fps, Engine.get_frames_per_second(), (t0 - capture_usec) / 1000.0, markers.size()])
		if traced:
			# The marker poses are baked with the head pose AT EXPOSURE TIME -- however long the
			# detection took, that fact does not age.
			print("[opencv_aruco] [aruco_marker_tracking::_detect_frame] flow #%d (7) detection done: detect_ms=%.1f markers=%d (play space; parking result for the main thread)" % [
					frame_id, detect_ms, markers.size()])
	_result_markers = markers
	_result_corners = corners
	_result_capture_usec = capture_usec
	_result_head_pose = head_pose
	_result_frame_id = frame_id


# --- Head pose ------------------------------------------------------------------------------

# The head pose in the XR PLAY SPACE (the space every XR tracker reports its "default" pose in,
# before the consumer applies world scale and reference frame). Reading the head tracker raw --
# instead of XRCamera3D.global_transform -- keeps the whole pipeline in that space, so the marker
# poses can be published on trackers verbatim and stay correct wherever the XROrigin3D sits and
# whatever center_on_hmd did to the reference frame.
func _head_pose_now() -> Transform3D:
	var head := XRServer.get_tracker(&"head") as XRPositionalTracker
	if head != null:
		var pose := head.get_pose(&"default")
		if pose != null and pose.has_tracking_data:
			return pose.transform
	# Flat/desktop fallback (OpenXR not initialised): derive a play-space pose from the active
	# camera. An XRCamera3D's global transform is XROrigin3D x reference_frame x play-space
	# pose (XRNode3D/XRCamera3D apply both on top of the raw pose), so strip those two again --
	# returning the world-space transform as-is would bake the origin into every published
	# tracker pose and the consuming XRAnchor3D would then apply it a second time. A camera
	# without an XROrigin3D ancestor is a plain flat-mode camera: world space == play space.
	# World scale is assumed 1, as everywhere else in this passthrough-AR setup.
	var cam := get_viewport().get_camera_3d()
	if cam == null:
		return Transform3D.IDENTITY
	var xform := cam.global_transform
	var ancestor: Node = cam.get_parent()
	while ancestor != null and not ancestor is XROrigin3D:
		ancestor = ancestor.get_parent()
	if ancestor != null:
		xform = XRServer.get_reference_frame().affine_inverse() \
				* (ancestor as Node3D).global_transform.affine_inverse() * xform
	return xform


# THE head-pose decision, for both backends. Two sources, in order of quality:
#   xrLocateSpace at the frame's capture time -- the runtime's fused, MEASURED estimate for a time
#     in the past. Returns play space natively, which is the space we want, so there is nothing to
#     convert. Needs an XrTime; the caller supplies one (0 = none available).
#   the pdt-stamped pose history -- interpolated forecasts. Always available, and the only option
#     off Quest, where neither the locator nor a predicted display time exists.
# A frame the runtime will not answer falls through to the history, counted so a bad stretch shows
# up in the trace instead of silently looking like a good one.
func _head_pose_at_capture(lookup_usec: int, xr_time_ns: int) -> Transform3D:
	if use_xr_locate_space and _head_locator != null and xr_time_ns != 0:
		var loc: Dictionary = _head_locator.locate_head(xr_time_ns)
		if loc.get("valid", false):
			# valid but not tracked = the runtime extrapolated through a tracking loss. Still the
			# best answer available, so it is used -- just labelled.
			_last_pose_source = "xrLocateSpace" if loc.get("tracked", false) else "xrLocateSpace_untracked"
			return loc["transform"]
		_locate_fallback_count += 1
	_last_pose_source = "history"
	return _head_pose_at(lookup_usec)


# Head pose at t_usec, interpolated between the two nearest history samples (the raw history
# has one sample per rendered frame; interpolating removes that quantisation).
# Falls back to the oldest/newest sample (or the live pose) at the edges of the history.
func _head_pose_at(t_usec: int) -> Transform3D:
	if _head_pose_history.is_empty():
		return _head_pose_now()
	if t_usec <= _head_pose_history[0][0]:
		return _head_pose_history[0][1]
	for i in range(_head_pose_history.size() - 1, -1, -1):
		if _head_pose_history[i][0] <= t_usec:
			if i == _head_pose_history.size() - 1:
				return _head_pose_history[i][1]
			var t0: int = _head_pose_history[i][0]
			var t1: int = _head_pose_history[i + 1][0]
			var w := clampf(float(t_usec - t0) / float(t1 - t0), 0.0, 1.0)
			var p0: Transform3D = _head_pose_history[i][1]
			var p1: Transform3D = _head_pose_history[i + 1][1]
			return p0.interpolate_with(p1, w)
	return _head_pose_history[0][1]


# Play space -> world. Play space is what XROrigin3D stands for, but a world-space transform
# additionally carries XRServer's reference frame (whatever center_on_hmd() last set) and the
# world scale. With an untouched XROrigin3D, no recentering and world_scale 1 all three factors
# are identity, so this costs nothing today and stops poses drifting off the moment locomotion or
# scaling appears.
# XRServer.world_origin rather than an XROrigin3D node reference: it is the same transform (the
# origin node publishes its global transform there) and it keeps this node free of scene-tree
# dependencies, which is the whole premise of the addon.
func _play_space_to_world(pose: Transform3D) -> Transform3D:
	var scaled := Transform3D(pose.basis, pose.origin * XRServer.world_scale)
	return XRServer.world_origin * XRServer.get_reference_frame() * scaled


# --- Desktop intrinsics fallback -------------------------------------------------------------

# Replace the Quest calibration with a pinhole guess for the camera we ACTUALLY got. Runs
# whenever the camera frame size changes, and only off Android.
#
# Why it is needed: the exported fx/fy/cx/cy are the left Quest passthrough camera's at 640x480
# and camera_distortion holds that lens's coefficients, so on a webcam all three are wrong in
# different ways. The principal point is the worst of them -- a 1280x720 frame centres at
# (640, 360), not (321, 240) -- and it skews the pose rather than merely scaling it; fx=437 for
# a 640-wide frame means a 72.6deg horizontal FOV, so on a typical laptop it is off by more than
# 2x; and distortion coefficients from a different lens ADD error instead of removing it.
#
# What the guess is: the textbook pinhole model -- principal point at the image centre, focal
# length from an assumed horizontal FOV, square pixels (fx == fy), zero distortion. That is
# enough to exercise detection, tracker publication and XRAnchor3D placement on a desktop.
# It is NOT a calibration: orientation tolerates a wrong focal length reasonably well, RANGE
# does not (the solved distance scales roughly linearly with the fx error). Run
# tools/cameraCalibration.py before trusting a number that came out of the desktop path.
#
# CALLER CONTRACT: main thread, and only while _detect_task_id == -1 -- the assignments below go
# through the write-through setters into properties a running detection task reads without a lock.
func _approximate_desktop_intrinsics(img: Image) -> void:
	var w := img.get_width()
	var h := img.get_height()
	# Recorded even when nothing below runs, so a size is reconsidered only when it changes
	# again. On Android that reduces the whole thing to one Vector2i compare per dispatch.
	_intrinsics_frame_size = Vector2i(w, h)
	# OS.get_name() is the platform this build is RUNNING on. NOTE this reads "not Android", not
	# "not a Quest" -- on a non-Quest Android device the exported Quest calibration would be kept
	# and be exactly as wrong as it is on a laptop. Fine here because the Quest is the only
	# Android target.
	if OS.get_name() == "Android":
		# The Quest: the exported values ARE this feed's calibration. Never overwritten.
		return
	# A CameraTexture hands out a 4x4 placeholder Image before the feed's first real frame.
	# Approximating from it gives fx ~ 3 px, which puts every marker about a millimetre from the
	# camera -- inside the near plane, so the anchors are clipped and nothing renders even though
	# detection, solvePnP and tracker publication all succeeded. Nothing this small can be a real
	# feed; the size compare at the call site brings us back when the real frame arrives.
	if w < 64 or h < 64:
		return
	var cx := w / 2.0
	var cy := h / 2.0
	var fx := cx / tan(deg_to_rad(DESKTOP_ASSUMED_HFOV_DEG) / 2.0)
	camera_intrinsics = Vector4(fx, fx, cx, cy)
	# Empty is the C++ side's "no distortion", which is a better assumption for an unknown lens
	# than another lens's measured coefficients.
	camera_distortion = PackedFloat64Array()
	# Loud on purpose, and not gated behind debug_prints_enabled: a silent approximation is how
	# someone measures a marker at 40cm, reads 80cm, and goes looking for a bug in solvePnP.
	push_warning(("[opencv_aruco] Not on Android: replaced the exported Quest calibration with a " +
			"pinhole GUESS for this %dx%d frame -- fx=fy=%.1f, cx=%.1f, cy=%.1f, no distortion, " +
			"assuming a %.0f deg horizontal FOV. Detection and tracker publication are testable " +
			"with this; marker RANGE is not. Calibrate with tools/cameraCalibration.py for real " +
			"numbers.") % [w, h, fx, cx, cy, DESKTOP_ASSUMED_HFOV_DEG])


func _exit_tree() -> void:
	# Stop the push camera FIRST, so no new frame can be dispatched while we tear down.
	if _android_cam != null and _android_cam_started:
		_android_cam.stop_camera()
	_pending_image = null
	_has_pending = false
	# A detection task may still be running on the pool; block until it is done so its bound
	# callable (which captures self) doesn't outlive the node. Costs at most one detection.
	if _detect_task_id != -1:
		WorkerThreadPool.wait_for_task_completion(_detect_task_id)
		_detect_task_id = -1
	# The processor is a plain Node that never enters the tree, so nothing frees it for us --
	# without this, every scene load leaks one OpenCVProcessor. AFTER the wait above: the
	# worker thread calls into it.
	if is_instance_valid(processor):
		processor.free()
	processor = null
	# Unregister every tracker, mirroring the engine's on_session_destroyed: pose invalidated
	# and state STOPPED before remove_tracker, for handlers that still hold a reference.
	for id in _trackers:
		var tracker: OpenXRMarkerTracker = _trackers[id]
		tracker.invalidate_pose(&"default")
		tracker.spatial_tracking_state = OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_STOPPED
		XRServer.remove_tracker(tracker)
	_trackers.clear()
