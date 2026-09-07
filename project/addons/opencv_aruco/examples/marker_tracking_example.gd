# Example consumer for the opencv_aruco addon -- both ways to use it, in one attachable
# script. Copy whichever route fits your app; delete the rest.
#
# Scene setup this example expects (any XR scene works):
#
#   YourSceneRoot (Node3D)          <- attach THIS script here
#     ArucoMarkerTracking           <- the addon's runtime node (add via "Add Child Node");
#                                      configure marker_sizes / default_marker_size in the
#                                      inspector -- index = ArUco id, 0 = "use default"
#     XROrigin3D                    <- your normal XR rig
#       XRCamera3D
#
# The markers are DICT_4X4_50 (ids 0-49). Camera calibration defaults fit the Quest 3
# passthrough camera; on other devices tune the exports on the ArucoMarkerTracking node.
extends Node3D

## Your XR rig's origin. Anchors MUST be children of it: a tracker pose is play-space, and
## an XRAnchor3D applies it as its LOCAL transform.
@export var xr_origin: XROrigin3D
## The addon's runtime node.
@export var marker_tracking: ArucoMarkerTracking

# tracker name -> XRAnchor3D spawned by route A.
var _anchors: Dictionary = {}


func _ready() -> void:
	# Fallback discovery so the example runs without wiring the exports: sibling nodes with
	# the default names.
	if xr_origin == null:
		xr_origin = get_node_or_null("XROrigin3D")
	if marker_tracking == null:
		marker_tracking = get_node_or_null("ArucoMarkerTracking")

	# ---------------------------------------------------------------------------------------
	# ROUTE A -- the standard Godot XR route (recommended).
	# Identical to the official "OpenXR spatial entities" tutorial pattern: nothing below
	# knows the markers come from OpenCV, and the same code works unchanged against Godot's
	# built-in OpenXR marker tracking on runtimes that support it.
	# ---------------------------------------------------------------------------------------
	XRServer.tracker_added.connect(_on_tracker_added)
	XRServer.tracker_removed.connect(_on_tracker_removed)
	# Trackers published before this node entered the tree won't fire tracker_added again:
	for tracker_name in XRServer.get_trackers(XRServer.TRACKER_ANCHOR):
		_on_tracker_added(tracker_name, XRServer.TRACKER_ANCHOR)

	# ---------------------------------------------------------------------------------------
	# ROUTE B (event half) -- the addon's id-keyed signal, once per applied detection result:
	# ---------------------------------------------------------------------------------------
	if marker_tracking != null:
		marker_tracking.markers_updated.connect(_on_markers_updated)

	# ---------------------------------------------------------------------------------------
	# ALTERNATIVE without any code: because this addon's tracker names are deterministic
	# ("openxr/spatial_entity/aruco_<id>"), you can author an XRAnchor3D in the editor as a
	# child of the XROrigin3D and set its `tracker` property to e.g.
	#   openxr/spatial_entity/aruco_3
	# It stays inactive until marker 3 is first seen, then follows it. (The real OpenXR
	# backend can't offer this -- its entity ids are assigned at runtime.)
	# ---------------------------------------------------------------------------------------


# --- ROUTE A: one XRAnchor3D per marker tracker ---------------------------------------------

func _on_tracker_added(tracker_name: StringName, type: int) -> void:
	if type != XRServer.TRACKER_ANCHOR:
		return
	var tracker := XRServer.get_tracker(tracker_name)
	# This class check is the whole "is it a marker?" filter -- works for any marker backend.
	if not tracker is OpenXRMarkerTracker:
		return
	if _anchors.has(tracker_name):
		return

	# The tracker is fully populated BEFORE tracker_added fires, so id/type/size/pose are
	# all readable right here:
	print("marker appeared: aruco id=%d physical size=%s m" % [tracker.marker_id, tracker.bounds_size])

	var anchor := XRAnchor3D.new()          # or your own marker scene with an XRAnchor3D root
	anchor.name = "marker_%d" % tracker.marker_id
	anchor.tracker = tracker_name           # binding by name; pose name becomes "default"
	anchor.show_when_tracked = true         # auto-hide while the marker is lost (pose paused)

	# Visualise: a flat box at the marker's real size (replace with your content).
	var mesh_instance := MeshInstance3D.new()
	mesh_instance.mesh = BoxMesh.new()
	mesh_instance.scale = Vector3(tracker.bounds_size.x, tracker.bounds_size.y, 0.01)
	anchor.add_child(mesh_instance)

	xr_origin.add_child(anchor)             # play-space pose => child of the origin
	_anchors[tracker_name] = anchor

	# Optional, per-tracker events instead of polling:
	#   tracker.pose_changed.connect(...)                  # every new detection (~12/s)
	#   tracker.pose_lost_tracking.connect(...)            # grace period elapsed -> paused
	#   tracker.spatial_tracking_state_changed.connect(...)# TRACKING <-> PAUSED <-> STOPPED


func _on_tracker_removed(tracker_name: StringName, _type: int) -> void:
	# Fires after a marker was gone for marker_stopped_timeout_s (default 10 s).
	if _anchors.has(tracker_name):
		print("marker gone: %s" % tracker_name)
		_anchors[tracker_name].queue_free()
		_anchors.erase(tracker_name)


# --- ROUTE B: id-keyed API on the ArucoMarkerTracking node ----------------------------------
# For logic that wants poses by ArUco id without touching tracker objects or anchor nodes
# (avatar rigs, gameplay checks, averaging over several markers).

func _on_markers_updated(ids: Array) -> void:
	print("this detection frame contained ids: ", ids)


func _process(_delta: float) -> void:
	if marker_tracking == null:
		return

	# Poses are PLAY-space: assign them to children of the XROrigin3D directly...
	if marker_tracking.markers_fresh([2, 3]):        # both tracked within the grace period?
		var pose := marker_tracking.get_average_marker_pose([2, 3])
		pass  # e.g.  $"../XROrigin3D/MyRig".transform = pose

	# ...or convert to WORLD space for nodes outside the rig:
	if marker_tracking.has_marker(5):                # ever seen (may be stale -- check age)
		var world_pose := marker_tracking.get_marker_world_pose(5)
		var age_ms := marker_tracking.marker_age_ms(5)
		pass  # e.g. use world_pose if age_ms < 200.0
