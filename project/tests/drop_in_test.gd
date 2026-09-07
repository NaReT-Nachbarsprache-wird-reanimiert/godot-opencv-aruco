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

	# Smoke check on the demo scene (_ready does not run since the instance never enters the
	# tree). load()/instantiate() succeed even on script parse errors, missing ext_resources
	# and assignments to renamed properties -- the engine drops those silently -- so assert the
	# OUTCOME: scripts actually attached, scene property overrides actually applied.
	var demo_scene: PackedScene = load("res://main_3d.tscn")
	_check(demo_scene != null, "demo scene resource loads")
	if demo_scene != null:
		var demo := demo_scene.instantiate()
		_check(demo != null, "demo scene instantiates")
		if demo != null:
			_check(demo.get_script() == load("res://main_3d.gd"), "demo root has its script attached")
			var amt := demo.get_node_or_null("ArucoMarkerTracking")
			_check(amt is ArucoMarkerTracking, "scene contains an ArucoMarkerTracking with the addon script")
			if amt is ArucoMarkerTracking:
				_check(amt.debug_prints_enabled and amt.tcp_stream_enabled,
						"scene overrides for debug_prints_enabled/tcp_stream_enabled applied")
				_check(amt.marker_sizes.size() == 10 and is_equal_approx(amt.marker_sizes[0], 0.1),
						"scene override for marker_sizes applied")
			_check(demo.get_node_or_null("XROrigin3D/XRCamera3D") != null
					and demo.get_node_or_null("CameraLayer/CameraPreview") != null,
					"demo @onready node paths exist in the scene")
			demo.free()
			# The instantiate above pushed the scene's debug_prints_enabled=true into the C++
			# static (property setter); reset it so the detection below logs nothing.
			OpenCVProcessor.set_debug_prints_enabled(false)

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
	var intrinsics := Vector4(img.get_width(), img.get_width(), img.get_width() / 2.0, img.get_height() / 2.0)
	# Identity camera pose: returned poses are camera-space. The test only needs a stable,
	# finite pose; metric correctness on device is the calibration exports' business.
	var detected: Dictionary = mt.processor.get_6dof_of_all_aruco_patches_from_godot_image(
			img, {0: MARKER_SIZE}, MARKER_SIZE, 1.0, intrinsics, PackedFloat64Array(),
			Transform3D.IDENTITY, {})
	_check(detected.has(0), "marker id 0 detected in the image")
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
