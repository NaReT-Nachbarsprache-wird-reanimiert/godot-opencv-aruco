# Drop-in replacement for Godot's built-in OpenXR marker tracking (XR_EXT_spatial_marker_tracking),
# backed by the opencv_aruco GDExtension and the Godot CameraServer instead of the OpenXR runtime.
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
# Usage: add this node anywhere in the scene (it has no scene-tree dependencies), configure the
# exports, consume markers via XRServer.tracker_added / XRAnchor3D as in the official
# "OpenXR spatial entities" tutorial. See res://addons/opencv_aruco/README.md.
class_name ArucoMarkerTracking
extends Node

# Tracker name prefix; full name = PREFIX + str(marker_id). Kept in sync with the class comment.
const TRACKER_NAME_PREFIX := "openxr/spatial_entity/aruco_"

## Emitted after a detection result was applied, with the ArUco ids seen in that frame. For
## one-shot reactions; polling the getters below from _process is equally fine.
signal markers_updated(ids: Array)
## Emitted once the camera feed is running and frames will start arriving. The demo uses it to
## show the feed in a TextureRect; get_camera_texture() serves late subscribers.
signal camera_feed_started(texture: CameraTexture)

## Master switch. Read once when entering the tree: the camera pipeline is only started while
## true. Toggling it off at runtime stops dispatching new detections (in-flight ones finish)
## and pauses every published tracker (pose invalidated, state PAUSED), like a lost marker.
@export var enabled := true

# --- Marker configuration -------------------------------------------------------------------
## Fallback physical side length, in meters, for every marker id WITHOUT a marker_sizes entry.
## Used as the solvePnP marker size (sets the pose's metric scale) AND published as the
## tracker's bounds_size, so the two can never disagree.
@export_range(0.01, 0.3, 0.001, "or_greater", "suffix:m") var default_marker_size := 0.1

## Ground-truth lookup table: index = marker id, value = physical side length in meters.
## 0 = unset -> that id falls back to default_marker_size (as do all ids >= the array length),
## so an untouched table behaves exactly like a single-size setup.
@export_range(0.01, 0.3, 0.001, "or_greater", "suffix:m") var marker_sizes: Array[float] = [
	0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05,
]

## How long a marker may be absent from detection results before its tracker's pose is
## invalidated and its spatial_tracking_state goes to PAUSED. Detection is noisy (motion blur,
## glancing angles drop a marker for a frame or two); half a second bridges the dropouts.
@export_range(0, 5000, 10.0, "suffix:ms") var marker_lost_timeout_ms := 500.0

## How long after the last detection a PAUSED tracker is fully removed from the XRServer
## (state STOPPED first, mirroring the engine's removal path). 0 disables removal: trackers
## then stay registered (paused) for the whole session, bounded at 50 by DICT_4X4_50.
@export_range(0, 120, 0.5, "suffix:s") var marker_stopped_timeout_s := 10.0

# --- Debugging ------------------------------------------------------------------------------
# Single switch for ALL debug output, this script's and the C++ extension's. Off by default.
# Errors are never gated and print regardless.
# Format for every debug line: "[opencv_aruco] [aruco_marker_tracking::function] event: k=v" --
# the fixed prefix is what makes them findable in logcat (adb logcat | grep opencv_aruco).
@export var debug_prints_enabled := false:
	set(value):
		debug_prints_enabled = value
		# Mirror every change into the C++ static right away. The extension gates its ACV_DBG
		# lines on a flag only GDScript can set; scene instantiation applies exported values
		# before _ready, so the flag is already correct when the (printing) C++ ctor runs.
		OpenCVProcessor.set_debug_prints_enabled(value)

## Stream every detected frame + its marker corners to tools/tcp_receiver.py (port 7007 via
## `adb reverse`). Debug tooling only -- leave off in production scenes.
@export var tcp_stream_enabled := false

# --- Camera calibration (left Quest passthrough camera "50", native 640x480 frame) ----------
# Exported so they can be tuned in the inspector instead of hunting through code. When NOT
# running on Android they are rewritten whenever the camera frame size changes
# (_approximate_desktop_intrinsics, at the same point in the frame as _sync_marker_sizes);
# otherwise read-only: detection tasks read them without a lock (same pattern as
# _marker_size_table), so treat inspector edits as pre-run configuration, not live tuning.
# Intrinsics (fx, fy, cx, cy) in pixels for the NATIVE frame (640x480 on the Quest).
# _detect_frame scales all four with the downscale factor at use time -- never bake that factor
# into these values.
@export var camera_intrinsics := Vector4(435.37335635, 435.96983202, 320.84589009, 241.55014114)
# OpenCV distCoeffs (k1, k2, p1, p2, k3) for the Quest passthrough lens; an EMPTY array means
# "no distortion".
@export var camera_distortion: PackedFloat64Array = [-0.00484306, 0.14036606, 0.00044449, -0.00108918, -0.29608385]
# Horizontal FOV assumed by the desktop pinhole guess (see _approximate_desktop_intrinsics).
# 65deg is the middle of the usual laptop-webcam range. Hardcoded rather than exported on
# purpose: it is the single number in a deliberately rough fallback, and a knob here would
# invite tuning it by eye instead of running tools/cameraCalibration.py.
const DESKTOP_ASSUMED_HFOV_DEG := 65.0
# Detection resolution knob: 1.0 = native frame, 0.5 = half width AND half height -> markedly
# cheaper detection, at the price of small or distant markers dropping below the resolution the
# detector needs. _detect_frame hands it to the C++ side AND scales the intrinsics above by the
# same factor; the two must always move together, which is why the factor belongs here and is
# never baked into camera_intrinsics.
@export_range(0.1, 1.0, 0.05) var image_downscale_factor := 1.0
# Physical passthrough-camera pose relative to the head/VIEW pose it gets combined with.
# MEASURED -- these are NOT the raw Camera2 metadata; see the warning below.
# The quaternion is ~168.8deg about X = the Android sensor->camera-optical 180deg X-flip PLUS
# the camera's real ~11deg pitch. The C++ marker pose already contains that same 180deg flip
# (its negate-Y/Z change of basis), so _ready multiplies by Quaternion(1,0,0,0) (=180deg about
# X) to cancel the flip and keep ONLY the physical mounting tilt (-> _lens_pose).
# The translation is in the sensor frame (X right, Y up, Z toward viewer), which matches Godot
# camera axes -> no sign flips.
#
# These two are ONE calibration and must be replaced as a PAIR -- a rotation from one solve
# beside a translation from another describes no camera that exists.
#
# DO NOT "fix" these by pasting in what the Quest's ACAMERA_LENS_POSE_ROTATION / _TRANSLATION
# report at startup, however authoritative that dump looks. It is gyro-referenced
# (LENS_POSE_REFERENCE == GYROSCOPE): the metadata describes the camera relative to the IMU,
# while the head pose it is combined with HERE is the VIEW pose. Those two frames differ by the
# IMU's mounting rotation, which NEITHER api exposes -- OpenXR has no IMU reference space and
# Camera2 never mentions the view -- so the difference cannot be looked up, only measured. That
# raw dump is what this file shipped until now, and it costs ~0.9deg: a standing offset of
# ~15mm at 1m, ~5mm at 40cm, which no pose-path change can touch.
#
# Provenance: solved by tools/handeye_solve.py from ~500 captured samples of marker 0
# (H_i * L * M_i collapsing to one world pose at 1.2mm median / 3.5mm p90), then verified in
# CAMERA PIXELS with the reprojection overlay -- on the android-camera-plugin branch, commit
# 81468ed. These are the values that branch RUNS with (its aruco_markers.tscn override), not
# the different, also-measured default sitting in its OpenCVProcessor.h.
# The lens pose describes the physical mount, so it is independent of how frames get delivered
# (CameraX push there, CameraServer readback here) and carries over unchanged -- PROVIDED the
# feed picked in _on_camera_feeds_updated is the same physical camera the capture ran on (the
# left Quest passthrough camera, id "50"). Re-measure with that tool rather than editing by eye.
@export var lens_rotation_raw := Quaternion(-0.9951163, -0.0028897487, 0.0037281485, 0.098596975)
@export var lens_translation := Vector3(-0.03352603, -0.017866991, -0.058882877)

# --- Capture-latency compensation -----------------------------------------------------------
## The Image get_image() returns is OLDER than "now": sensor -> ISP -> CameraServer texture
## takes 1-2 camera frames. Pairing those old pixels with the LIVE head pose bakes an error
## proportional to head speed, so a fresh detection first "drags" with the head, then settles.
## We keep a short timestamped head-pose history and sample the head pose from camera_latency_ms
## ago instead; that pose travels with the frame, and the C++ side bakes the markers straight to
## play space with it. This readback path has NO sensor timestamp, so this fixed guess is the
## only correction available -- tune on device: marker still drags WITH the head -> raise;
## marker lags behind the real one during motion -> lower.
@export_range(0, 300, 1.0, "or_greater", "suffix:ms") var camera_latency_ms := 90.0

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
# holding its last good pose through a dropout needs the pose to outlive the tracker.
# DICT_4X4_50 bounds this at 50 entries, so it cannot grow without limit.
var _marker_poses: Dictionary = {}
# id -> resolved size in meters for the C++ side. Rebuilt by _sync_marker_sizes, which runs in
# _ready and then once per dispatch in _process -- but ONLY while no detection task is in
# flight, so the unlocked read in _detect_frame never overlaps a write.
var _marker_size_table: Dictionary = {}

# Godot has a native CameraServer (Camera2) backend on Android since 4.5, so
# CameraServerExtension is only needed on desktop (Windows). Keep this var UNTYPED and
# instantiate via ClassDB so the script still parses on platforms where that class isn't
# registered.
var _camera_extension
var _cam_texture: CameraTexture
# Frame size camera_intrinsics has already been reconciled with; ZERO = none yet. On Android it
# only records the size (the exported Quest calibration is the right one and is never touched);
# off Android it is the size the pinhole guess was computed for. Compared per dispatch rather
# than latched once, because a CameraTexture hands out a 4x4 PLACEHOLDER Image before the feed's
# first real frame -- a guess derived from that is nonsense, and latched it would stay nonsense
# for the whole session.
var _intrinsics_frame_size := Vector2i.ZERO

# --- Detection worker -----------------------------------------------------------------------
# On the Quest the OpenCV detection costs ~80ms, which run synchronously would cap the whole
# app at ~10fps. We run ONLY the detection (detectMarkers + solvePnP) off the main thread, as
# one-shot WorkerThreadPool tasks -- Godot owns the threads, so there is no Thread/Mutex/
# Semaphore lifecycle to manage here. At most ONE task is in flight at a time; get_image()
# and all XRServer/tracker writes stay on the main thread.
# There is no pending-frame slot: frames are PULLED here, so instead of parking a frame while
# the worker is busy we simply do not read one back (see _process).
var _detect_task_id := -1              # WorkerThreadPool task id; -1 = no task in flight
# output slots, written by the task; the main thread reads them only AFTER
# wait_for_task_completion(), which is the synchronization point (no lock needed)
var _result_markers: Dictionary = {}
# id -> PackedVector2Array of the 4 marker corners in the frame's own pixel space, straight
# from the C++ detector. Debug data for the TCP overlay ONLY.
var _result_corners: Dictionary = {}

# Derived ONCE from the exported raw lens values in _ready (before any detection task exists).
var _lens_pose := Transform3D.IDENTITY

# [t_usec, play-space head Transform3D] pairs, newest last; main thread only.
var _head_pose_history: Array = []

# --- TCP debug streamer (tools/tcp_receiver.py) ---------------------------------------------
const TCP_HOST := "127.0.0.1"
const TCP_PORT := 7007			# view available ports with adb reverse --list

var _stream_peer: StreamPeerTCP
var _tcp_reconnect_timer := 0.0
var _last_tcp_status := -1
var _tcp_send_timer := 0.0
const TCP_SEND_INTERVAL := 0.01
# The frame the in-flight (or just finished) detection task is working on, kept so the streamer
# can send it TOGETHER with that detection's corners -- the corners only exist once the worker
# is done, so a frame sent at readback time could never carry them. Overwritten per dispatch.
var _stream_img: Image


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

## Last known pose converted to WORLD space (applies the XR reference frame and the current
## world origin, i.e. the XROrigin3D's global transform; world scale is assumed 1 as in any
## passthrough-AR setup). IDENTITY if never detected.
func get_marker_world_pose(id: int) -> Transform3D:
	if not _marker_poses.has(id):
		return Transform3D.IDENTITY
	return XRServer.world_origin * XRServer.get_reference_frame() * _marker_poses[id]

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

## The live camera texture once the feed is running, else null. camera_feed_started announces
## the moment it becomes available.
func get_camera_texture() -> CameraTexture:
	return _cam_texture

# --- Capability compat helpers ---------------------------------------------------------------
# Mirror OpenXRSpatialMarkerTrackingCapability's support queries for code that feature-checks
# before subscribing. The C++ detector is built with DICT_4X4_50, hence ArUco only.

func is_aruco_supported() -> bool:
	return true

func is_qrcode_supported() -> bool:
	return false

func is_micro_qrcode_supported() -> bool:
	return false

func is_april_tag_supported() -> bool:
	return false

#######################################################################################################

# Single source of truth for a marker id's physical size: table entry if present and set,
# default_marker_size otherwise. The bounds check doubles as the guard for arrays the inspector
# resized to fewer/more elements.
func _marker_size_for(id: int) -> float:
	if id >= 0 and id < marker_sizes.size():
		var s: float = marker_sizes[id]
		if s > 0.0:
			return s
	return default_marker_size


# Re-resolve marker_sizes/default_marker_size into the id -> size table the C++ side gets, and
# put the same numbers on the live trackers' bounds_size. Both consumers of a marker's size are
# refreshed here, so they can never drift apart.
# CALLER CONTRACT: main thread, and only while _detect_task_id == -1. _detect_frame reads
# _marker_size_table from a worker thread without a lock, so this is the one point in the frame
# at which rewriting it is safe. Cheap enough to run per dispatch (~12x/s).
func _sync_marker_sizes() -> void:
	for id in marker_sizes.size():
		_marker_size_table[id] = _marker_size_for(id)
	# Rows the inspector shrank the array past must go, else a deleted entry would keep feeding
	# solvePnP its old size instead of falling back to default_marker_size. keys() is a copy,
	# so erasing inside the loop is safe.
	for id in _marker_size_table.keys():
		if id >= marker_sizes.size():
			_marker_size_table.erase(id)
	# Live trackers got their bounds_size at publish time; without this a size change would
	# only reach consumers after the marker had been lost and re-published. Compared
	# approximately: bounds_size round-trips through 32-bit floats.
	for id in _trackers:
		var size := _marker_size_for(id)
		var tracker: OpenXRMarkerTracker = _trackers[id]
		if not is_equal_approx(tracker.bounds_size.x, size):
			tracker.bounds_size = Vector2(size, size)
			if debug_prints_enabled:
				print("[opencv_aruco] [aruco_marker_tracking::_sync_marker_sizes] bounds resized: id=%d size=%.3f" % [id, size])

#######################################################################################################

func _ready() -> void:
	# The property setter already pushed this into the extension at scene-instantiation time;
	# repeat it here so the flag is also correct when the scene does NOT override the default
	# (the setter never fires then) and a previous run left the static true. Still BEFORE
	# new(): the C++ constructor prints (OpenCV build info + Quest intrinsics dump).
	OpenCVProcessor.set_debug_prints_enabled(debug_prints_enabled)
	processor = OpenCVProcessor.new()

	# Lens pose from the exported raw Camera2 values (see their declarations for the why of the
	# extra 180deg X-flip). Built once here, read by detection tasks without a lock afterwards.
	_lens_pose = Transform3D(Basis((lens_rotation_raw * Quaternion(1, 0, 0, 0)).inverse()), lens_translation)

	# Resolve the size table for the C++ side (entries are pre-resolved through
	# _marker_size_for, so the C++ default only fires for ids >= the table length). _process
	# refreshes it from here on.
	_sync_marker_sizes()

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

	if OS.get_name() == "Android":
		# Quest: request camera access; the native CameraServer surfaces feeds once granted.
		OS.request_permission("android.permission.CAMERA")
		OS.request_permission("horizonos.permission.HEADSET_CAMERA")
	elif ClassDB.class_exists("CameraServerExtension"):
		# Desktop (Windows): custom backend that registers the webcam as a feed.
		_camera_extension = ClassDB.instantiate("CameraServerExtension")  # keep reference alive

	# Since Godot 4.5, monitoring_feeds must be true before feeds are enumerated.
	CameraServer.monitoring_feeds = true
	CameraServer.camera_feeds_updated.connect(_on_camera_feeds_updated)
	_on_camera_feeds_updated()                          # in case a feed is already present

	if tcp_stream_enabled:
		_connect_tcp()


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

####################################################################################################

func _process(_delta: float) -> void:
	if tcp_stream_enabled:
		_poll_tcp(_delta)
		# Ticked here rather than in the readback branch below: the debug frame is sent from
		# _poll_detection_task, which the early returns further down never reach.
		_tcp_send_timer += _delta

	# (a) Sample the head pose EVERY render frame, before any early return below: _head_pose_at
	# interpolates between the two nearest samples, so its accuracy is bounded by the sampling
	# period. Recording only on detection frames would coarsen the history from ~14ms to ~80ms
	# and put most of the capture-latency compensation back as error.
	var now_usec := Time.get_ticks_usec()
	_head_pose_history.append([now_usec, _head_pose_now()])
	while _head_pose_history.size() > 1 and _head_pose_history[0][0] < now_usec - 500_000:
		_head_pose_history.pop_front()

	# (b) Collect the latest finished detection and publish it (main thread -> XRServer and
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
	if _cam_texture == null:
		return

	# (c) Hand the newest camera frame to a detection task. get_image() (the GPU->CPU readback)
	# and the head-pose lookup must happen on the main thread; the task only does OpenCV work.
	#
	# Readback ONLY when no detection is running. Detection costs ~80ms while _process runs at
	# the render rate (~72fps on Quest), so an unconditional get_image() paid the full readback
	# ~6x per detection and threw all but the last one away. The readback is a GPU->CPU stall
	# on the main thread, i.e. render-frame time burned for nothing. Skipping it while a task
	# runs also means the frame we DO read back is the freshest one at the instant detection
	# starts, which shortens the pose-history lookback. This is also why there is no
	# pending-frame slot: frames are pulled here, so declining to pull IS the frame drop.
	if _detect_task_id != -1:
		return

	# Past that guard nothing can be reading _marker_size_table on a worker thread, which makes
	# this the only safe place to rewrite it -- so this is where an inspector edit to
	# marker_sizes/default_marker_size on a RUNNING remote deploy reaches both solvePnP and the
	# published bounds. See _sync_marker_sizes.
	_sync_marker_sizes()

	var readback_t0 := Time.get_ticks_usec()
	var img := _cam_texture.get_image()
	if img == null:
		return
	# format lookup table https://docs.godotengine.org/en/stable/classes/class_image.html#enum-image-format
	if debug_prints_enabled:
		print("[opencv_aruco] [aruco_marker_tracking::_process] readback_ms=%.2f image_format=%d" % [(Time.get_ticks_usec() - readback_t0) / 1000.0, img.get_format()])

	# The exported calibration belongs to the Quest passthrough lens; on any other camera it is
	# simply wrong, so derive a pinhole guess from the frame we just read back. Re-checked
	# whenever the frame SIZE changes rather than once -- see _intrinsics_frame_size. HERE and
	# not in _ready because it needs that size, and here and nowhere else in _process because
	# past the _detect_task_id guard -- beside _sync_marker_sizes, for the same reason -- is the
	# one point in the frame at which no worker thread can be reading the intrinsics.
	if _intrinsics_frame_size != Vector2i(img.get_width(), img.get_height()):
		_approximate_desktop_intrinsics(img)

	# Head pose AT capture time: NOT the live pose -- the pixels in img are ~camera_latency_ms
	# old (passthrough pipeline), so look that far back in the history filled in (a). The pose
	# travels with the frame and the C++ side applies it together with the lens pose, so the
	# markers come back in play space.
	var capture_usec := now_usec - int(camera_latency_ms * 1000.0)
	_start_detection_task(img, _head_pose_at(capture_usec))


# Replace the Quest calibration with a pinhole guess for the camera we ACTUALLY got. Runs
# whenever the camera frame size changes, and only off Android.
#
# Why it is needed: the exported fx/fy/cx/cy are the left Quest passthrough camera's at 640x480
# and camera_distortion holds that lens's coefficients, so on a webcam all three are wrong in
# different ways. The principal point is the worst of them -- a 1280x720 frame centres at
# (640, 360), not (320, 241) -- and it skews the pose rather than merely scaling it; fx=435 for
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
# CALLER CONTRACT: main thread, and only while _detect_task_id == -1 -- exactly as for
# _sync_marker_sizes, because _detect_frame reads camera_intrinsics and camera_distortion from a
# worker thread without a lock.
func _approximate_desktop_intrinsics(img: Image) -> void:
	var w := img.get_width()
	var h := img.get_height()
	# Recorded even when nothing below runs, so a size is reconsidered only when it changes
	# again. On Android that reduces the whole thing to one Vector2i compare per dispatch.
	_intrinsics_frame_size = Vector2i(w, h)
	# OS.get_name() is the platform this build is RUNNING on: "Android" on the Quest, "Windows"
	# on the desktop dev machine. NOTE this reads "not Android", not "not a Quest" -- on a
	# non-Quest Android device the exported Quest calibration would be kept and be exactly as
	# wrong as it is on a laptop. Fine here because the Quest is the only Android target.
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
	# Empty is the C++ side's "no distortion" (see the export's declaration), which is a better
	# assumption for an unknown lens than another lens's measured coefficients.
	camera_distortion = PackedFloat64Array()
	# Loud on purpose, and not gated behind debug_prints_enabled: a silent approximation is how
	# someone measures a marker at 40cm, reads 80cm, and goes looking for a bug in solvePnP.
	push_warning(("[opencv_aruco] Not on Android: replaced the exported Quest calibration with a " +
			"pinhole GUESS for this %dx%d frame -- fx=fy=%.1f, cx=%.1f, cy=%.1f, no distortion, " +
			"assuming a %.0f deg horizontal FOV. Detection and tracker publication are testable " +
			"with this; marker RANGE is not. Calibrate with tools/cameraCalibration.py for real " +
			"numbers.") % [w, h, fx, cx, cy, DESKTOP_ASSUMED_HFOV_DEG])


# The head pose in the XR PLAY SPACE (the space every XR tracker reports its "default" pose
# in, before the consumer applies world scale and reference frame). Reading the head tracker
# raw -- instead of XRCamera3D.global_transform like the pre-addon code -- keeps the whole
# pipeline in that space, so the marker poses can be published on trackers verbatim and stay
# correct wherever the XROrigin3D sits and whatever center_on_hmd did to the reference frame.
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


# Main thread only. If the in-flight task has finished: clean it up and apply its result.
func _poll_detection_task() -> void:
	if _detect_task_id == -1 or not WorkerThreadPool.is_task_completed(_detect_task_id):
		return
	# Mandatory cleanup of every finished task; returns immediately here (the task is done) and
	# doubles as the memory barrier that makes the task's _result_markers write visible to us.
	WorkerThreadPool.wait_for_task_completion(_detect_task_id)
	_detect_task_id = -1
	_apply_detection_result()
	# Only now do the frame and its corners both exist, so this is the earliest point at which
	# the overlay can be streamed as one consistent pair.
	if tcp_stream_enabled:
		_stream_detected_frame()


func _start_detection_task(img: Image, cam_xform: Transform3D) -> void:
	# Held for the debug streamer: _poll_detection_task sends THIS frame once the task below
	# has produced the corners that belong to it.
	_stream_img = img
	_detect_task_id = WorkerThreadPool.add_task(_detect_frame.bind(img, cam_xform),
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
	var size := _marker_size_for(id)
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
# already in PLAY space -- baked with the head pose at the frame's capture time, which
# travelled with the frame -- so publishing is a plain assignment.
func _apply_detection_result() -> void:
	var markers: Dictionary = _result_markers
	var now_usec := Time.get_ticks_usec()
	var seen_ids: Array = []
	for id in markers:
		_marker_poses[id] = markers[id]        # the id-keyed record the public API serves
		_marker_last_seen[id] = now_usec
		_publish_marker(id, markers[id])
		seen_ids.append(id)

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
			# cleanly if the marker comes back. Bounded at 50 entries by DICT_4X4_50.
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
# off the main thread. Touches only `processor`, read-only config and the _result_markers
# slot -- never the scene tree or XRServer. Writing _result_markers without a lock is safe: the
# main thread reads the slot only after wait_for_task_completion() on this task.
func _detect_frame(img: Image, cam_xform: Transform3D) -> void:
	# No conversion: the C++ side handles 1ch (Quest Y-plane), 3ch (RGB), and 4ch (RGBA).
	var t0 := Time.get_ticks_usec()
	# The intrinsics are for the native frame -- whatever get_image() returns, 640x480 on the
	# Quest -- and all four components scale with the image, so image_downscale_factor is applied
	# at use time.
	var intrinsics := camera_intrinsics * image_downscale_factor
	# The two transforms that hold for EVERY marker -- head pose at capture time and physical
	# lens offset -- combined into ONE camera->play-space pose. The C++ side pre-multiplies it
	# onto each solvePnP pose, so the returned Dictionary is already in PLAY space.
	var cam_to_play := cam_xform * _lens_pose
	# Out-parameter for the debug overlay: Dictionaries are shared references in Godot, so the
	# C++ side writes the detected pixel corners into THIS instance. Built fresh per detection
	# (rather than clearing _result_corners) so the main thread can never see a half-filled
	# dictionary -- the slot is only re-pointed at the end, past the same barrier as
	# _result_markers.
	var corners: Dictionary = {}
	var markers: Dictionary = processor.get_6dof_of_all_aruco_patches_from_godot_image(img, _marker_size_table, default_marker_size, image_downscale_factor, intrinsics, camera_distortion, cam_to_play, corners)
	# Guarded inline rather than via a helper function: a helper would build this string on
	# every detection (~12x/s on the Quest) before it could check the flag.
	if debug_prints_enabled:
		var detect_ms := (Time.get_ticks_usec() - t0) / 1000.0
		var tracking_fps := 1000.0 / detect_ms if detect_ms > 0.0 else 0.0
		print("[opencv_aruco] [aruco_marker_tracking::_detect_frame] detect_ms=%.1f tracking_fps=%.1f render_fps=%d markers=%d" % [
				detect_ms, tracking_fps, Engine.get_frames_per_second(), markers.size()])
	_result_markers = markers
	_result_corners = corners


# Head pose at t_usec, interpolated between the two nearest history samples (the raw history
# has one sample per rendered frame, ~14ms at 72fps; interpolating removes that quantisation).
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


func _exit_tree() -> void:
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

####################################################################################################
# --- TCP debug streamer ---------------------------------------------------------------------

# Send the last detected frame plus its corners, at most every TCP_SEND_INTERVAL. Called from
# _poll_detection_task, i.e. once per finished detection (~12/s on Quest) rather than once per
# render frame -- the interval only throttles further, it can no longer force a send.
func _stream_detected_frame() -> void:
	if _stream_img == null:
		return
	if _tcp_send_timer < TCP_SEND_INTERVAL:
		return
	_tcp_send_timer = 0.0
	if _stream_peer != null and _stream_peer.get_status() == StreamPeerTCP.STATUS_CONNECTED:
		_send_frame_tcp(_stream_img, _result_corners)


# Wire format, big-endian throughout (_stream_peer.big_endian), consumed by
# tools/tcp_receiver.py:
#   header:  width u32, height u32, image_format u32, payload_size u32      (16 bytes)
#   payload: payload_size raw Image bytes
#   markers: marker_count u32, then per marker id u32 + 8 f32               (36 bytes each)
#            = the 4 corners as x0,y0,x1,y1,x2,y2,x3,y3 in the payload's own pixel space.
# The marker block is APPENDED after the image so the original 16-byte header stayed as it
# was. Corner count per marker is fixed at 4 (guaranteed by the C++ side), hence no per-marker
# length.
func _send_frame_tcp(img: Image, corners: Dictionary) -> void:
	if _stream_peer == null:
		return

	_stream_peer.poll()

	if _stream_peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		return

	var bytes: PackedByteArray = img.get_data()

	_stream_peer.put_u32(img.get_width())
	_stream_peer.put_u32(img.get_height())
	_stream_peer.put_u32(img.get_format())
	_stream_peer.put_u32(bytes.size())

	var err := _stream_peer.put_data(bytes)
	if err != OK:
		# Bail out before the marker block: the receiver is reading a fixed number of bytes per
		# frame, so appending to a truncated payload would desync every following frame too.
		push_error("[opencv_aruco] [aruco_marker_tracking::_send_frame_tcp] put_data failed: err=%d" % err)
		return

	_stream_peer.put_u32(corners.size())
	for id in corners:
		_stream_peer.put_u32(id)
		var pts: PackedVector2Array = corners[id]
		for p in pts:
			_stream_peer.put_float(p.x)
			_stream_peer.put_float(p.y)


func _connect_tcp() -> void:
	_stream_peer = StreamPeerTCP.new()
	_stream_peer.big_endian = true

	var err := _stream_peer.connect_to_host(TCP_HOST, TCP_PORT)
	if debug_prints_enabled:
		print("[opencv_aruco] [aruco_marker_tracking::_connect_tcp] connect_to_host: err=%d" % err)


func _poll_tcp(delta: float) -> void:
	if _stream_peer == null:
		_connect_tcp()
		return

	_stream_peer.poll()

	var status := _stream_peer.get_status()

	if status != _last_tcp_status:
		if debug_prints_enabled:
			print("[opencv_aruco] [aruco_marker_tracking::_poll_tcp] status changed: from=%d to=%d" % [_last_tcp_status, status])
		_last_tcp_status = status

	if status == StreamPeerTCP.STATUS_CONNECTED:
		_tcp_reconnect_timer = 0.0
		return

	if status == StreamPeerTCP.STATUS_CONNECTING:
		return

	if status == StreamPeerTCP.STATUS_ERROR or status == StreamPeerTCP.STATUS_NONE:
		_tcp_reconnect_timer += delta
		if _tcp_reconnect_timer >= 1.0:
			_tcp_reconnect_timer = 0.0
			if debug_prints_enabled:
				print("[opencv_aruco] [aruco_marker_tracking::_poll_tcp] reconnecting")
			_connect_tcp()
