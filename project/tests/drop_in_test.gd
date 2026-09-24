# Headless functional test for the opencv_aruco addon's drop-in contract. Run with:
#   godot --headless --path project res://tests/drop_in_test.tscn
# Exit code 0 = all checks passed. Covers the full chain: OpenCV detection on the checked-in
# test image (marker id 0, DICT_4X4_50) -> tracker publication -> consumption through the
# standard XR route (XRServer signals + XRAnchor3D) -> pause/remove lifecycle.
#
# The ArucoMarkerTracking node runs with enabled = false: no camera pipeline (headless has no
# feed and macOS would prompt for camera permission), the detection result is produced by
# calling the C++ detector directly and injected through the same _result_markers /
# _apply_detection_result path a finished worker task uses.
extends Node

const MARKER_SIZE := 0.05
const TRACKER_NAME := "openxr/spatial_entity/aruco_0"

var _failures: PackedStringArray = []
# [name, type] pairs captured by the XRServer signal handlers, plus the tracker completeness
# snapshot taken INSIDE the tracker_added handler (the drop-in contract says the tracker must
# be fully populated before add_tracker).
var _added_events: Array = []
var _removed_events: Array = []
var _added_snapshot := {}


func _check(cond: bool, what: String) -> void:
	if cond:
		print("  ok: " + what)
	else:
		_failures.append(what)
		printerr("  FAIL: " + what)


func _on_tracker_added(tracker_name: StringName, type: int) -> void:
	_added_events.append([tracker_name, type])
	if tracker_name == StringName(TRACKER_NAME):
		var t := XRServer.get_tracker(tracker_name)
		_added_snapshot = {
			"is_marker_tracker": t is OpenXRMarkerTracker,
			"marker_id": t.marker_id if t is OpenXRMarkerTracker else -1,
			"marker_type": t.marker_type if t is OpenXRMarkerTracker else -1,
			"has_pose": (t as XRPositionalTracker).has_pose(&"default"),
		}


func _on_tracker_removed(tracker_name: StringName, type: int) -> void:
	_removed_events.append([tracker_name, type])


func _ready() -> void:
	print("[drop_in_test] engine surface")
	_check(ClassDB.class_exists("OpenXRMarkerTracker"), "OpenXRMarkerTracker class exists")
	_check(ClassDB.class_exists("OpenCVProcessor"), "OpenCVProcessor GDExtension class exists")

	# Smoke check on the app scene (_ready does not run since the instance never enters the
	# tree). load()/instantiate() succeed even on script parse errors, missing ext_resources
	# and assignments to renamed properties -- the engine drops those silently -- so assert the
	# OUTCOME: scripts actually attached, scene property overrides actually applied.
	var app_scene: PackedScene = load("res://cpr_trainer.tscn")
	_check(app_scene != null, "app scene resource loads")
	if app_scene != null:
		var app := app_scene.instantiate()
		_check(app != null, "app scene instantiates")
		if app != null:
			_check(app.get_script() == load("res://cpr_trainer.gd"), "app root has its script attached")
			var amt := app.get_node_or_null("ArucoMarkerTracking")
			_check(amt is ArucoMarkerTracking, "scene contains an ArucoMarkerTracking with the addon script")
			if amt is ArucoMarkerTracking:
				# Anchored on marker_dictionary, not on a marker size. This check exists only to
				# prove an override ARRIVED -- instantiate() silently drops assignments to renamed
				# or removed properties, so the outcome has to be asserted rather than the act --
				# and for that it needs a property whose value is a DECISION, not a MEASUREMENT.
				# marker_sizes[0] stood here and had to be edited every time someone re-measured a
				# printed marker, which is how it came to disagree with the scene. Which dictionary
				# the markers were PRINTED in cannot drift like that.
				# NOTE this pins the test to a scene whose app runs 4x4_50. A demo scene on the
				# default dictionary (36h12) cannot express the override at all -- Godot omits
				# default values from .tscn -- so that branch needs its own value here.
				_check(amt.marker_dictionary == OpenCVProcessor.MARKER_DICT_4X4_50,
						"scene override for marker_dictionary applied")
				# The measurement rig is demo-only and attaches by signal, so its absence is silent
				# by design -- which is exactly why the scene has to assert it is there.
				_check(amt.get_node_or_null("TcpDebugStream") != null
						and amt.get_node_or_null("DetectionDiagnostics") != null,
						"app scene carries both diagnostics nodes under the tracking node")
			_check(app.get_node_or_null("XROrigin3D/XRCamera3D") != null
					and app.get_node_or_null("CameraLayer/CameraPreview") != null,
					"root's @onready node paths exist in the scene")
			app.free()
			# Whatever debug_prints_enabled the scene carries, the instantiate above pushed it
			# into the C++ static (property setter); reset it so the detection below logs nothing.
			OpenCVProcessor.set_debug_prints_enabled(false)

	# The addon's exports are pushed into the C++ properties in _ready, so whichever literals sit
	# in the GDScript win for every consumer -- silently, since a wrong calibration announces
	# nothing. The two sets are kept identical precisely so that push is a no-op, and this is the
	# check that keeps them that way.
	print("[drop_in_test] calibration defaults (GDScript exports == C++ defaults)")
	var ref_proc := OpenCVProcessor.new()          # C++ defaults, never pushed to
	var plain := ArucoMarkerTracking.new()
	plain.enabled = false
	add_child(plain)                               # _ready -> _push_calibration()
	var pushed := plain.get_processor()
	_check(pushed != null, "untouched node built its processor")
	if pushed != null:
		_check(pushed.camera_intrinsics.is_equal_approx(ref_proc.camera_intrinsics),
				"camera_intrinsics default survives the push")
		_check(Array(pushed.camera_distortion) == Array(ref_proc.camera_distortion),
				"camera_distortion default survives the push")
		_check(pushed.lens_rotation_raw.is_equal_approx(ref_proc.lens_rotation_raw),
				"lens_rotation_raw default survives the push")
		_check(pushed.lens_translation.is_equal_approx(ref_proc.lens_translation),
				"lens_translation default survives the push")
		_check(is_equal_approx(pushed.aruco_patch_size, ref_proc.aruco_patch_size)
				and Array(pushed.aruco_patch_sizes) == Array(ref_proc.aruco_patch_sizes),
				"marker size defaults survive the push")
		_check(is_equal_approx(pushed.image_downscale_factor, ref_proc.image_downscale_factor),
				"image_downscale_factor default survives the push")
		_check(pushed.marker_dictionary == ref_proc.marker_dictionary,
				"marker_dictionary default survives the push")
	ref_proc.free()
	plain.free()

	XRServer.tracker_added.connect(_on_tracker_added)
	XRServer.tracker_removed.connect(_on_tracker_removed)

	var mt := ArucoMarkerTracking.new()
	mt.enabled = false                      # no camera pipeline, see the file comment
	mt.marker_sizes = [MARKER_SIZE]         # id 0 -> MARKER_SIZE
	add_child(mt)                           # _ready builds processor, lens pose, size table

	# The camera fallback of _head_pose_now must return PLAY-space poses: with an XROrigin3D
	# away from identity, the XRCamera3D's world transform contains the origin offset, and
	# feeding that into the pipeline would double-apply the offset on every XRAnchor3D
	# consumer. Only checkable while no live head tracker overrides the fallback.
	var head := XRServer.get_tracker(&"head") as XRPositionalTracker
	var head_active: bool = head != null and head.get_pose(&"default") != null \
			and head.get_pose(&"default").has_tracking_data
	if not head_active:
		print("[drop_in_test] head-pose camera fallback")
		var fb_origin := XROrigin3D.new()
		add_child(fb_origin)
		fb_origin.global_transform = Transform3D(Basis(), Vector3(10, 0, 5))
		var fb_cam := XRCamera3D.new()
		fb_origin.add_child(fb_cam)
		fb_cam.current = true
		_check(mt._head_pose_now().origin.is_equal_approx(Vector3.ZERO),
				"camera fallback strips the XROrigin3D transform (play space)")
		fb_origin.free()

	print("[drop_in_test] OpenCV detection on the test image")
	var img := Image.load_from_file("res://assets/img_of_marker0_dict4x4_50.png")
	_check(img != null and not img.is_empty(), "test image loads")
	# The asset is DICT_4X4_50 while the addon defaults to ARUCO_MIP_36h12, so this doubles as
	# the test of the selectable dictionary: without the switch nothing is found at all.
	mt.marker_dictionary = OpenCVProcessor.MARKER_DICT_4X4_50
	# Synthetic pinhole intrinsics for this image, no distortion, no downscale -- all through
	# the write-through setters, so this exercises those too.
	mt.camera_intrinsics = Vector4(img.get_width(), img.get_width(),
			img.get_width() / 2.0, img.get_height() / 2.0)
	mt.camera_distortion = PackedFloat64Array()
	mt.image_downscale_factor = 1.0
	# Neutralise the lens pose so the poses come back in pure CAMERA space. detect_markers
	# applies head_pose * lens_pose, and the raw quaternion is decoded as
	# (raw * Quaternion(1,0,0,0)).inverse() -- Quaternion(1,0,0,0) is 180deg about X, whose
	# square is identity, so this exact value is the one that cancels. The test only needs a
	# stable, finite pose; metric correctness on device is the calibration's business.
	mt.lens_rotation_raw = Quaternion(1, 0, 0, 0)
	mt.lens_translation = Vector3.ZERO
	var corners: Dictionary = {}
	var detected: Dictionary = mt.get_processor().detect_markers(img, Transform3D.IDENTITY, corners)
	_check(detected.has(0), "marker id 0 detected in the image")
	_check(corners.has(0) and (corners[0] as PackedVector2Array).size() == 4,
		"corners_out received 4 pixel corners for id 0")
	if not detected.has(0):
		_finish()
		return
	var pose: Transform3D = detected[0]
	_check(pose.origin.is_finite() and pose.origin != Vector3.ZERO, "detected pose is finite and non-zero")

	print("[drop_in_test] tracker publication (worker-result path)")
	mt._result_markers = detected
	mt._apply_detection_result()

	var tracker := XRServer.get_tracker(TRACKER_NAME) as OpenXRMarkerTracker
	_check(tracker != null, "tracker registered under '%s'" % TRACKER_NAME)
	if tracker == null:
		_finish()
		return
	_check(tracker.type == XRServer.TRACKER_ANCHOR, "tracker type is TRACKER_ANCHOR")
	_check(tracker.marker_id == 0, "marker_id is 0")
	_check(tracker.marker_type == OpenXRSpatialComponentMarkerList.MARKER_TYPE_ARUCO, "marker_type is MARKER_TYPE_ARUCO")
	_check(tracker.bounds_size.is_equal_approx(Vector2(MARKER_SIZE, MARKER_SIZE)), "bounds_size matches the configured size")
	_check(tracker.spatial_tracking_state == OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_TRACKING, "state is TRACKING")
	var xr_pose := tracker.get_pose(&"default")
	_check(xr_pose != null and xr_pose.has_tracking_data, "pose 'default' has tracking data")
	_check(xr_pose != null and xr_pose.transform.is_equal_approx(pose), "pose transform matches the detection")
	_check(_added_events.has([StringName(TRACKER_NAME), XRServer.TRACKER_ANCHOR]), "tracker_added fired with TRACKER_ANCHOR")
	_check(_added_snapshot.get("is_marker_tracker", false), "tracker was an OpenXRMarkerTracker inside the tracker_added handler")
	_check(_added_snapshot.get("marker_id", -1) == 0 and _added_snapshot.get("marker_type", -1) == OpenXRSpatialComponentMarkerList.MARKER_TYPE_ARUCO and _added_snapshot.get("has_pose", false),
			"tracker was fully populated (id/type/pose) before tracker_added")
	_check(mt.get_marker_tracker(0) == tracker, "get_marker_tracker(0) returns the registered tracker")
	_check(mt.get_marker_pose(0).is_equal_approx(pose), "get_marker_pose(0) matches")
	_check(mt.has_marker(0) and mt.markers_fresh([0]) and mt.markers_ever_seen([0]), "id-keyed freshness API agrees")

	print("[drop_in_test] standard-route consumption (XRAnchor3D)")
	var origin := XROrigin3D.new()
	add_child(origin)
	var anchor := XRAnchor3D.new()
	anchor.tracker = TRACKER_NAME           # binds immediately -- the tracker already exists
	anchor.show_when_tracked = true
	origin.add_child(anchor)
	_check(anchor.get_is_active(), "anchor bound to the tracker")
	_check(anchor.get_has_tracking_data(), "anchor sees tracking data")
	_check(anchor.transform.is_equal_approx(pose), "anchor follows the published pose (world scale 1, no reference frame)")
	# show_when_tracked only toggles visibility while a primary XR interface exists
	# (XRNode3D::_update_visibility) -- headless has none, so assert the underlying
	# has_tracking_data contract above and check visibility only when it can change.
	var visibility_active := XRServer.primary_interface != null
	if visibility_active:
		_check(anchor.visible, "anchor visible while tracked")

	print("[drop_in_test] momentary loss -> PAUSED, tracker kept")
	mt._marker_last_seen[0] = Time.get_ticks_usec() - int(0.6 * 1_000_000)  # > 500ms grace
	mt._result_markers = {}
	mt._apply_detection_result()
	_check(XRServer.get_tracker(TRACKER_NAME) == tracker, "tracker still registered while paused")
	_check(tracker.spatial_tracking_state == OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_PAUSED, "state is PAUSED")
	_check(not tracker.get_pose(&"default").has_tracking_data, "pose invalidated")
	_check(tracker.get_pose(&"default").transform.is_equal_approx(pose), "last pose kept through the pause (consumers may hold it)")
	_check(not anchor.get_has_tracking_data(), "anchor lost tracking data while paused")
	if visibility_active:
		_check(not anchor.visible, "anchor hidden while paused (show_when_tracked)")
	_check(mt.has_marker(0) and not mt.markers_fresh([0]), "id-keyed API: known but stale")

	print("[drop_in_test] long absence -> STOPPED + removed")
	mt._marker_last_seen[0] = Time.get_ticks_usec() - int((mt.marker_stopped_timeout_s + 1.0) * 1_000_000)
	mt._result_markers = {}
	mt._apply_detection_result()
	_check(XRServer.get_tracker(TRACKER_NAME) == null, "tracker removed from the XRServer")
	_check(tracker.spatial_tracking_state == OpenXRSpatialEntityTracker.ENTITY_TRACKING_STATE_STOPPED, "state is STOPPED for handlers still holding a reference")
	_check(_removed_events.has([StringName(TRACKER_NAME), XRServer.TRACKER_ANCHOR]), "tracker_removed fired")
	_check(mt.get_marker_tracker(0) == null, "get_marker_tracker(0) is null after removal")
	_check(mt.has_marker(0), "last pose record outlives the tracker")

	print("[drop_in_test] re-detection -> fresh tracker, anchor rebinds by name")
	mt._result_markers = detected
	mt._apply_detection_result()
	var tracker2 := XRServer.get_tracker(TRACKER_NAME) as OpenXRMarkerTracker
	_check(tracker2 != null and tracker2 != tracker, "a NEW tracker object was registered")
	_check(anchor.get_has_tracking_data(), "anchor rebound automatically and tracks again")

	_finish()


func _finish() -> void:
	if _failures.is_empty():
		print("[drop_in_test] PASSED (all checks)")
		get_tree().quit(0)
	else:
		printerr("[drop_in_test] FAILED: %d check(s)" % _failures.size())
		for f in _failures:
			printerr("  - " + f)
		get_tree().quit(1)
