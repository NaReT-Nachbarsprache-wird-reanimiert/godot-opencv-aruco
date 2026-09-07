# Demo consumer for the opencv_aruco addon. Deliberately uses ONLY the standard Godot XR
# marker-tracking route -- XRServer.tracker_added/tracker_removed + XRAnchor3D bound by tracker
# name -- exactly like the "spatial entities manager" in the official OpenXR spatial entities
# tutorial. Nothing in here knows where the trackers come from: swap ArucoMarkerTracking for
# Godot's built-in OpenXR marker tracking (or run both) and this script keeps working, which is
# the whole point of the addon.
#
# Per tracked marker it spawns an XRAnchor3D under the XROrigin3D with a box mesh sized from the
# tracker's bounds_size. The anchor follows the tracker's "default" pose by itself;
# show_when_tracked hides the box while the marker is lost (the addon invalidates the pose after
# its grace period), and tracker_removed frees it.
extends Node3D

# Rendered thickness (z) of a marker box in meters -- x/y come from the tracker's bounds_size,
# so the box overlays the printed marker at true size.
const PATCH_THICKNESS := 0.01

# tracker name -> XRAnchor3D. Keyed by name because that is what the XRServer signals carry;
# an anchor is only ever created in _on_tracker_added (which checks this dict first) and only
# ever removed together with its map entry, so one tracker can never own two anchors.
var _anchors: Dictionary = {}
# ONE unit-cube mesh shared by every anchor box; per-marker size lives in the MeshInstance3D's
# scale, never in the mesh.
var _patch_mesh := BoxMesh.new()

@onready var xr_origin: XROrigin3D = $XROrigin3D
@onready var cam_preview: TextureRect = $CameraLayer/CameraPreview
@onready var marker_tracking: ArucoMarkerTracking = $ArucoMarkerTracking


func _ready() -> void:
	XRServer.tracker_added.connect(_on_tracker_added)
	XRServer.tracker_removed.connect(_on_tracker_removed)
	# Trackers published before this node entered the tree (scene reloads) never fire
	# tracker_added again -- pick them up from the server's current registry.
	for tracker_name in XRServer.get_trackers(XRServer.TRACKER_ANCHOR):
		_on_tracker_added(tracker_name, XRServer.TRACKER_ANCHOR)

	# Debug feed preview. The texture is created once the camera feed is up; the getter covers
	# the case where that already happened before this _ready ran.
	marker_tracking.camera_feed_started.connect(_on_camera_feed_started)
	if marker_tracking.get_camera_texture() != null:
		_on_camera_feed_started(marker_tracking.get_camera_texture())


func _on_camera_feed_started(texture: CameraTexture) -> void:
	cam_preview.texture = texture


func _on_tracker_added(tracker_name: StringName, type: int) -> void:
	if type != XRServer.TRACKER_ANCHOR:
		return
	var tracker := XRServer.get_tracker(tracker_name)
	# The type/class checks are the whole "is this a marker?" filter -- by design there is no
	# name matching here, so trackers from ANY marker backend (this addon, or the engine's own
	# spatial entities capability) get a box.
	if not tracker is OpenXRMarkerTracker:
		return
	if _anchors.has(tracker_name):
		return

	var anchor := XRAnchor3D.new()
	# Cosmetic (remote scene tree); identity is the dict key. Tracker names contain '/', which
	# node names cannot, hence the marker id instead.
	anchor.name = "aruco_patch_%d" % tracker.marker_id
	anchor.tracker = tracker_name       # also resets the anchor's pose name to "default"
	anchor.show_when_tracked = true     # hidden while the pose is invalidated (marker lost)

	var mesh_instance := MeshInstance3D.new()
	mesh_instance.mesh = _patch_mesh
	# The BoxMesh is a unit cube and the tracked pose is rigid (scale 1), so this local scale
	# IS the box's size in meters.
	var bounds: Vector2 = tracker.bounds_size
	mesh_instance.scale = Vector3(bounds.x, bounds.y, PATCH_THICKNESS)
	anchor.add_child(mesh_instance)

	# Under the (stationary) XROrigin3D: the anchor applies the tracker's play-space pose as its
	# local transform, so it must be a direct child of the origin to land in the right place.
	xr_origin.add_child(anchor)
	_anchors[tracker_name] = anchor


func _on_tracker_removed(tracker_name: StringName, _type: int) -> void:
	if not _anchors.has(tracker_name):
		return
	_anchors[tracker_name].queue_free()
	_anchors.erase(tracker_name)


func _process(_delta: float) -> void:
	# Keep box sizes in sync with the trackers' bounds_size, so an inspector edit to the
	# addon's marker size table on a RUNNING remote deploy reaches the rendered boxes too.
	# Compared approximately because scale components are 32-bit floats.
	for tracker_name in _anchors:
		var tracker := XRServer.get_tracker(tracker_name) as OpenXRMarkerTracker
		if tracker == null:
			continue
		var mesh_instance: MeshInstance3D = _anchors[tracker_name].get_child(0)
		var bounds: Vector2 = tracker.bounds_size
		if not is_equal_approx(mesh_instance.scale.x, bounds.x):
			mesh_instance.scale = Vector3(bounds.x, bounds.y, PATCH_THICKNESS)
