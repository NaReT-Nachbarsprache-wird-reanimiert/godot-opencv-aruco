extends Node3D
# DEBUG: draws a coloured XYZ axis on each ArUco marker so you can see its full 6DOF pose live --
# red = X, green = Y, blue = Z. A marker's gizmo is shown ONLY while that marker is actually being
# detected; markers that are out of view (or not present at all, like a removed navel marker) have
# their gizmo hidden, so no stray axis floats in front of you.
#
# MARKER SOURCE: this took an array of aruco_patch Node3Ds and read their transforms and their
# last_detected_ms metadata. Markers come from the opencv_aruco addon by ID now, so there are no
# patch nodes to point at -- and nothing to hide either, which is why the old
# hide_placeholder_boxes switch is gone. The gizmos are built here from marker_ids.
#
# WORLD space (get_marker_world_pose), because these gizmos are placed with global_transform. The
# addon's get_marker_pose() is PLAY space and would sit wrong by the XROrigin3D transform.

## The addon node that publishes the markers.
@export var marker_tracking: ArucoMarkerTracking
## Which marker ids to draw an axis for. One gizmo is created per entry, in this order.
@export var marker_ids: Array[int] = [0, 1, 2]
@export var axis_length := 0.1        # metres
@export var axis_thickness := 0.006   # metres
# A marker counts as detected when the addon reported it this recently.
@export var fresh_ms := 300

var _fresh := MarkerFreshness.new()
var _gizmos: Array[Node3D] = []


func _ready() -> void:
	_fresh.tracking_loss_timeout_ms = fresh_ms
	_fresh.attach(marker_tracking, marker_ids)
	for i in marker_ids.size():
		var g := _make_axis()
		g.visible = false
		add_child(g)
		_gizmos.append(g)


func _process(_delta: float) -> void:
	# Match AvatarRig's fusion set exactly: a marker can remain freshness-eligible on its own, but
	# it is visualized only when it belongs to the single newest camera result. This prevents a
	# remembered old gizmo from looking as though it participated in the current fusion. Both sides
	# get that set from the same place now (MarkerFreshness) rather than each re-deriving it.
	var shown := _fresh.result_ids()

	for i in marker_ids.size():
		var id: int = marker_ids[i]
		var g := _gizmos[i]
		if g == null:
			continue
		var used_in_newest_result := shown.has(id)
		g.visible = used_in_newest_result
		if used_in_newest_result:
			g.global_transform = marker_tracking.get_marker_world_pose(id)


func _make_axis() -> Node3D:
	var root := Node3D.new()
	root.add_child(_bar(Vector3(1, 0, 0), Color(1.0, 0.18, 0.18)))   # X red
	root.add_child(_bar(Vector3(0, 1, 0), Color(0.2, 1.0, 0.35)))    # Y green
	root.add_child(_bar(Vector3(0, 0, 1), Color(0.3, 0.55, 1.0)))    # Z blue
	return root


func _bar(axis: Vector3, col: Color) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var bm := BoxMesh.new()
	var t := axis_thickness
	var l := axis_length
	bm.size = Vector3(
		l if axis.x > 0.5 else t,
		l if axis.y > 0.5 else t,
		l if axis.z > 0.5 else t)
	mi.mesh = bm
	mi.position = axis * (l * 0.5)     # start at origin, extend along +axis
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = col
	mat.no_depth_test = true           # draw over passthrough so it's always visible
	mi.material_override = mat
	return mi
