extends Node3D
## Places the avatar from the newest fused ArUco result.
##
## Pipeline: newest markers -> common pose -> medoid/dead zones/smoothing/prior -> avatar.
##
## MARKER SOURCE. This rig used to hold three Node3D references (aruco_patch0/1/2) that the old
## main_3d.gd wrote poses into, and it recovered "which markers belong to the newest result" by
## comparing per-node last_detected_ms metadata for exact equality. The pipeline is the
## opencv_aruco addon now: markers are addressed by ID through ArucoMarkerTracking, which publishes
## each one as a real OpenXRMarkerTracker and carries a markers_updated signal whose argument is
## exactly the set of ids from one detection. MarkerFreshness wraps that; see its class comment.
##
## SPACE -- the one thing to get right when reading this against the old version. The addon's
## get_marker_pose() returns PLAY space, get_marker_world_pose() returns WORLD space. This rig
## applies its result with global_transform and the bundled marker offsets were measured against
## world-space marker poses, so the world-space getter is the correct one. Using the play-space one
## would compile, run, and be wrong by the XROrigin3D transform (zero only while that origin sits
## at identity, which is exactly why the mistake survives a desk test).

## The addon node that publishes the markers. Wired in the scene; nothing else here reaches into
## the tree for tracking data.
@export var marker_tracking: ArucoMarkerTracking

@export_group("Markers")
## ArUco ids of the three markers glued to the mannequin. These replace the old aruco_patch node
## references -- the ids are what the calibration offsets are keyed by, so changing one here means
## re-running the calibration for it.
@export var common_marker_id := 0
@export var chest_marker_id := 1
@export var torso_marker_id := 2

@export_group("Look")
@export_range(0.0, 0.95, 0.05) var avatar_transparency := 0.6
@export var avatar_tint := Color.WHITE

@export_group("Calibration")
@export var xr_controller_right: XRController3D
## Relearns the session rest pose without replacing the permanent marker offsets.
@export var relevel_button := "primary_click"

@export_group("Placement")
@export var enable_nudge := false
@export var target: Node3D
@export var nudge_speed := 0.05
@export var xr_controller_left: XRController3D
## Supplies the Quest floor height (has_floor / floor_height_world). The floor is a one-sided
## boundary only: it prevents penetration but never replaces the marker-measured height.
@export var floor_provider: Node

# Version the writable copy so this build starts from the validated previous calibration instead
# of silently loading either of the older on-device calibration files.
const SAVE_PATH := "user://navel_calibration_20260905.cfg"
const DEFAULT_CALIBRATION_PATH := "res://default_navel_calibration.cfg"

# Runtime filter values selected by tune_filter.py on the 2026-09-04 labelled session
# (calibration 1788433311 + stationary 1788430472 + moving 1788431407 at 617.5 mm):
# movement retained 99.7%, endpoint error 4.8 mm, return error 10.4 mm.
const FILTER_WINDOW := 5
const FILTER_POSITION_DEAD_ZONE_M := 0.020
const FILTER_ROTATION_DEAD_ZONE_DEG := 0.3
const FILTER_SMOOTHING_TIME_S := 1.2
const FILTER_PRIOR_TIME_S := 2.0
# Endpoint stability is evaluated over a complete measurement-target window.
const ENDPOINT_STABLE_POSITION_M := 0.0015
const ENDPOINT_STABLE_ROTATION_DEG := 1.0
const ENDPOINT_STABLE_DETECTIONS := 3
# The floor comes from the same grid run; the small-move band (10-30 mm) has no direct
# labelled evidence yet, so 40 mm is the safest value the data could not distinguish.
const REANCHOR_MIN_POSITION_M := 0.040
const REANCHOR_MIN_ROTATION_DEG := 3.0

var _common_provider := CommonPoseProvider.new()
var _filter := SimplePoseStabilizer.new()
var _fresh := MarkerFreshness.new()
var _marker_ids: Array = []
var _was_nudging := false
var _tracking_was_available := false
var _application_paused := false
var _runtime_initialized := false
var _mesh_floor_offset_ready := false
var _lowest_mesh_vertex_offset_y := 0.0


func _ready() -> void:
	visible = false
	_marker_ids = [common_marker_id, chest_marker_id, torso_marker_id]
	_fresh.attach(marker_tracking, _marker_ids)
	_filter.configure(
		FILTER_WINDOW,
		FILTER_POSITION_DEAD_ZONE_M,
		FILTER_ROTATION_DEAD_ZONE_DEG,
		FILTER_SMOOTHING_TIME_S,
		FILTER_PRIOR_TIME_S,
		ENDPOINT_STABLE_POSITION_M,
		ENDPOINT_STABLE_ROTATION_DEG,
		ENDPOINT_STABLE_DETECTIONS,
		REANCHOR_MIN_POSITION_M,
		REANCHOR_MIN_ROTATION_DEG
	)
	_apply_look()
	_common_provider.orientation_settled.connect(_on_orientation_settled)
	_load_calibration()
	if xr_controller_right != null:
		xr_controller_right.button_pressed.connect(_on_button)
	_runtime_initialized = true


func _notification(what: int) -> void:
	if not _runtime_initialized:
		return
	if what == NOTIFICATION_APPLICATION_PAUSED:
		_application_paused = true
		visible = false
	elif what == NOTIFICATION_APPLICATION_RESUMED:
		_application_paused = false
		# The XR world frame and camera stream may have changed while the Quest menu owned focus.
		# Never display or interpolate from the pre-pause world pose. The newest detection result
		# ages out on its own (MarkerFreshness stamps wall-clock time, which ran during the pause),
		# so nothing here has to invalidate it by hand.
		_tracking_was_available = false
		_common_provider.recalibrate_orientation()
		_filter.reset()
		visible = false
		print("Common pose: app resumed; collecting a fresh session rest pose.")


func _process(delta: float) -> void:
	# Ids from the single newest detection, already narrowed to this rig's markers and already
	# emptied if that result has aged past the freshness window.
	var result_ids := _fresh.result_ids()
	if result_ids.is_empty():
		_hold_after_tracking_loss()
	else:
		_update_tracking(result_ids, _fresh.result_ms(), delta)

	# Once a pose is confirmed, keep it visible through marker loss and reacquisition.
	# Startup, app resume, and re-levelling still require a newly confirmed rest pose.
	visible = (
		not _application_paused
		and _filter.is_ready()
		and _common_provider.has_rest_pose()
	)
	_update_nudge(delta)


func _load_calibration() -> void:
	if _common_provider.load_from(SAVE_PATH, _marker_ids):
		print("Common pose: calibration loaded from disk.")
	elif _common_provider.load_from(DEFAULT_CALIBRATION_PATH, _marker_ids):
		print("Common pose: bundled default calibration loaded.")
	else:
		push_error("Common pose: no user or bundled marker calibration is available.")


func _update_tracking(ids: Array, detection_ms: int, delta: float) -> void:
	_tracking_was_available = true
	# WORLD space, deliberately: see the space note in the class comment.
	var marker_poses := {}
	for id in ids:
		marker_poses[id] = marker_tracking.get_marker_world_pose(id)

	var raw_pose := _common_provider.get_pose(marker_poses, detection_ms)
	if not _common_provider.is_ready():
		return

	var filtered_pose := _filter.update(
		raw_pose,
		delta,
		detection_ms,
		_common_provider.rest_pose(),
		_common_provider.has_rest_pose()
	)
	# Re-anchor only after the complete measurement-only target window is stationary and its
	# displacement exceeds the separately configured relocation threshold.
	var reanchor_position := _filter.position_reanchor_ready()
	var reanchor_rotation := _filter.rotation_reanchor_ready()
	if _common_provider.reanchor_rest_from_stable_target(
		_filter.measurement_target(),
		detection_ms,
		reanchor_position,
		reanchor_rotation
	):
		_filter.complete_rest_reanchor(reanchor_position, reanchor_rotation)
	if _filter.is_ready() and _common_provider.has_rest_pose():
		_apply_filtered_pose(filtered_pose)


## Place the rig at the ArUco pose, then treat the Quest floor as a boundary: if the avatar's
## lowest mesh point would sink below the floor, raise the rig by exactly the penetration depth.
## An avatar above the floor is left untouched, preserving the marker-to-mannequin alignment.
func _apply_filtered_pose(pose: Transform3D) -> void:
	global_transform = pose
	if floor_provider == null or target == null:
		return
	if not floor_provider.has_floor():
		return
	if not _mesh_floor_offset_ready:
		var measured_lowest := _lowest_mesh_world_y(target)
		if not is_finite(measured_lowest):
			return
		_lowest_mesh_vertex_offset_y = measured_lowest - global_position.y
		_mesh_floor_offset_ready = true
		print(
			"Floor boundary: exact lowest-vertex offset %.3f m."
			% _lowest_mesh_vertex_offset_y
		)
	var lowest := global_position.y + _lowest_mesh_vertex_offset_y
	var floor_y: float = floor_provider.floor_height_world()
	if lowest < floor_y:
		global_position.y += floor_y - lowest


## World-space bottom from actual triangle vertices. Transforming an AABB's eight corners is
## conservative and included empty space in this rotated GLB, causing an unnecessary ~18 mm lift.
func _lowest_mesh_world_y(node: Node3D) -> float:
	var lowest := INF
	var instances := node.find_children("*", "MeshInstance3D", true, false)
	if node is MeshInstance3D:
		instances.append(node)
	for instance in instances:
		var mesh_instance := instance as MeshInstance3D
		if mesh_instance.has_meta("cpr_feedback"):
			continue
		if mesh_instance.mesh == null:
			continue
		for vertex in mesh_instance.mesh.get_faces():
			lowest = minf(lowest, (mesh_instance.global_transform * vertex).y)
	return lowest


## On complete loss, freeze the avatar and discard measurements from before the gap.
func _hold_after_tracking_loss() -> void:
	if not _tracking_was_available:
		return
	_tracking_was_available = false
	_filter.clear_measurement_history()


func _on_button(button_name: String) -> void:
	print("Right controller: ", button_name)
	if button_name != relevel_button:
		return
	# Asked per marker rather than against the newest result: re-levelling only needs the common
	# marker to be currently visible, not to have been part of the very last detection.
	if not _fresh.is_fresh(common_marker_id):
		print("Common pose: calibration needs common visibly.")
		return

	_common_provider.recalibrate_orientation()
	visible = false
	_filter.reset()
	print("Common pose: re-levelling; collecting 30+3 detection checkpoints.")


func _on_orientation_settled() -> void:
	# Save only marker-local offsets; the world-space rest pose is session-local.
	_common_provider.save_to(SAVE_PATH)
	# Show the robust rest first, but keep the genuine recent measurements collected while hidden.
	_filter.anchor_at(_common_provider.rest_pose())
	print("Common pose: session orientation settled; marker offsets saved.")


func _update_nudge(delta: float) -> void:
	if not enable_nudge or target == null:
		return

	var nudge := _read_nudge(delta)
	if nudge != Vector3.ZERO:
		target.position += nudge
		_mesh_floor_offset_ready = false
		_was_nudging = true
	elif _was_nudging:
		print("Final mannequin offset: ", target.position)
		_was_nudging = false


func _read_nudge(delta: float) -> Vector3:
	var direction := Vector3.ZERO
	if xr_controller_left != null:
		var left_stick: Vector2 = xr_controller_left.get_vector2("primary")
		direction.x += left_stick.x
		direction.y += left_stick.y
	if xr_controller_right != null:
		direction.z += -xr_controller_right.get_vector2("primary").y
	return direction * nudge_speed * delta


func _apply_look() -> void:
	var alpha := clampf(1.0 - avatar_transparency, 0.05, 1.0)
	for node in find_children("*", "MeshInstance3D", true, false):
		var mesh_instance := node as MeshInstance3D
		if mesh_instance.has_meta("cpr_feedback"):
			continue
		for surface in mesh_instance.get_surface_override_material_count():
			var material := mesh_instance.get_active_material(surface)
			if material is BaseMaterial3D:
				var copy := material.duplicate() as BaseMaterial3D
				copy.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
				copy.albedo_color = Color(avatar_tint, alpha)
				mesh_instance.set_surface_override_material(surface, copy)
