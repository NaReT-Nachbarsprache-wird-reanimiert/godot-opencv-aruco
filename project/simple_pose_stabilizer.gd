class_name SimplePoseStabilizer
extends RefCounted
## Stabilization order:
##   1. Select the medoid of recent independent detections.
##   2. Apply position and rotation dead zones.
##   3. Smooth toward the selected measurement.
##   4. If enabled, pull gently toward session rest.

var window := 3
var position_dead_zone_m := 0.005
var rotation_dead_zone_deg := 1.5
var smoothing_time_s := 0.5
var prior_time_s := 1.0

# A stable endpoint is checked over a complete target-history window. A separate minimum distance
# from remembered rest prevents stationary measurement bias from being accepted as relocation.
var endpoint_stable_position_m := 0.002
var endpoint_stable_rotation_deg := 0.5
var endpoint_stable_detections := 7
var reanchor_min_position_m := 0.0
var reanchor_min_rotation_deg := 0.0

# Converts rotation into equivalent point displacement inside the medoid score.
# 0.20 m was selected by the 2026-09-04 grid run; 1 degree scores like ~3.5 mm of position.
var medoid_rotation_radius_m := 0.20

# When the target corroborates rest but the display has not settled onto it (right after a
# re-anchor), the dead zone would park the display short and leave the remainder to the slow
# prior. Landing closes onto rest at smoothing speed until within these margins.
const LANDING_POSITION_M := 0.001
const LANDING_ROTATION_DEG := 0.1

var _stable_pose := Transform3D.IDENTITY
var _target_pose := Transform3D.IDENTITY
var _recent_poses: Array[Transform3D] = []
var _last_detection_ms := -1
var _ready := false
var _hold_anchor_once := false
var _reacquiring := false
var _has_measurement_target := false
var _endpoint_targets: Array[Transform3D] = []
var _position_reanchor_ready := false
var _rotation_reanchor_ready := false


func configure(
		p_window: int,
		p_position_dead_zone_m: float,
		p_rotation_dead_zone_deg: float,
		p_smoothing_time_s: float,
		p_prior_time_s: float = 1.0,
		p_endpoint_stable_position_m: float = 0.002,
		p_endpoint_stable_rotation_deg: float = 0.5,
		p_endpoint_stable_detections: int = 7,
		p_reanchor_min_position_m: float = 0.0,
		p_reanchor_min_rotation_deg: float = 0.0
) -> void:
	window = maxi(p_window, 1)
	position_dead_zone_m = maxf(p_position_dead_zone_m, 0.0)
	rotation_dead_zone_deg = maxf(p_rotation_dead_zone_deg, 0.0)
	smoothing_time_s = maxf(p_smoothing_time_s, 0.0)
	prior_time_s = maxf(p_prior_time_s, 0.0)
	endpoint_stable_position_m = maxf(p_endpoint_stable_position_m, 0.0)
	endpoint_stable_rotation_deg = maxf(p_endpoint_stable_rotation_deg, 0.0)
	endpoint_stable_detections = maxi(p_endpoint_stable_detections, 1)
	reanchor_min_position_m = maxf(p_reanchor_min_position_m, 0.0)
	reanchor_min_rotation_deg = maxf(p_reanchor_min_rotation_deg, 0.0)
	while _recent_poses.size() > window:
		_recent_poses.pop_front()


func update(
		raw_pose: Transform3D,
		delta_s: float,
		detection_ms: int = -1,
		rest_pose: Transform3D = Transform3D.IDENTITY,
		use_rest_prior: bool = false
) -> Transform3D:
	var accepted_detection := _accept_new_detection(raw_pose, detection_ms)
	if accepted_detection and use_rest_prior:
		_update_stable_endpoint_state(rest_pose)
	# After a complete tracking gap, hold the existing avatar until a full fresh medoid window is
	# available. This prevents one bad first reacquired detection from moving the avatar.
	if _reacquiring:
		return _stable_pose

	# The first visible frame after rest confirmation must be exactly the robust rest pose.
	if _hold_anchor_once:
		_hold_anchor_once = false
		return _stable_pose

	if not _ready:
		_stable_pose = _target_pose
		_ready = true
		return _stable_pose

	var amount := _smoothing_amount(delta_s)
	var next_position := _smoothed_position(amount)
	var next_rotation := _smoothed_rotation(amount)

	if use_rest_prior:
		next_position = _landing_position(next_position, amount, rest_pose)
		next_rotation = _landing_rotation(next_rotation, amount, rest_pose)

	if use_rest_prior and prior_time_s > 0.0:
		var pull := 1.0 - exp(-maxf(delta_s, 0.0) / prior_time_s)
		# The weak prior remains active during movement; smoothing is ten times faster at the current
		# values, so the measurement leads. A confirmed stable endpoint then becomes the new rest.
		next_position = next_position.lerp(rest_pose.origin, pull)
		next_rotation = next_rotation.normalized().slerp(
			rest_pose.basis.get_rotation_quaternion().normalized(),
			pull
		).normalized()

	_stable_pose = Transform3D(Basis(next_rotation), next_position)
	return _stable_pose


## Add at most one sample per OpenCV result, never one sample per render frame.
func _accept_new_detection(raw_pose: Transform3D, detection_ms: int) -> bool:
	if detection_ms < 0 or detection_ms == _last_detection_ms:
		return false

	_last_detection_ms = detection_ms
	_recent_poses.append(raw_pose)
	while _recent_poses.size() > window:
		_recent_poses.pop_front()
	if _reacquiring:
		if _recent_poses.size() < window:
			return true
		_set_measurement_target(_medoid())
		# The avatar stays visible at its held pose while collecting this window.
		# Resume normal smoothing from that pose once the new target is confirmed.
		_reacquiring = false
		return true

	# Before the window fills, use the newest measurement instead of a weak partial medoid.
	_set_measurement_target(_medoid() if _recent_poses.size() == window else raw_pose)
	return true


## A target is stationary only when every target in the complete recent window remains close to
## the newest one. Checking the whole range prevents a slow continuous movement from passing merely
## because each individual step is small.
func _update_stable_endpoint_state(rest_pose: Transform3D) -> void:
	if not _has_measurement_target:
		return
	_endpoint_targets.append(_target_pose)
	while _endpoint_targets.size() > endpoint_stable_detections:
		_endpoint_targets.pop_front()
	if _endpoint_targets.size() < endpoint_stable_detections:
		return

	var newest_rotation := _target_pose.basis.get_rotation_quaternion().normalized()
	var position_is_stable := true
	var rotation_is_stable := true
	for target in _endpoint_targets:
		if target.origin.distance_to(_target_pose.origin) > endpoint_stable_position_m:
			position_is_stable = false
		if rad_to_deg(
			target.basis.get_rotation_quaternion().normalized().angle_to(newest_rotation)
		) > endpoint_stable_rotation_deg:
			rotation_is_stable = false

	_position_reanchor_ready = (
		position_is_stable
		and _target_pose.origin.distance_to(rest_pose.origin)
			> maxf(position_dead_zone_m, reanchor_min_position_m)
	)
	_rotation_reanchor_ready = (
		rotation_is_stable
		and rad_to_deg(
			newest_rotation.angle_to(rest_pose.basis.get_rotation_quaternion().normalized())
		) > maxf(rotation_dead_zone_deg, reanchor_min_rotation_deg)
	)


func _set_measurement_target(pose: Transform3D) -> void:
	_has_measurement_target = true
	_target_pose = pose


## Close onto rest at smoothing speed while the target agrees with rest but the display sits off
## it. Lerping toward rest rather than the target keeps measurement jitter out of the landing.
func _landing_position(current: Vector3, amount: float, rest_pose: Transform3D) -> Vector3:
	if _target_pose.origin.distance_to(rest_pose.origin) > position_dead_zone_m:
		return current
	if current.distance_to(rest_pose.origin) <= LANDING_POSITION_M:
		return current
	return current.lerp(rest_pose.origin, amount)


func _landing_rotation(current: Quaternion, amount: float, rest_pose: Transform3D) -> Quaternion:
	var rest_rotation := rest_pose.basis.get_rotation_quaternion().normalized()
	var target_rotation := _target_pose.basis.get_rotation_quaternion().normalized()
	if rad_to_deg(target_rotation.angle_to(rest_rotation)) > rotation_dead_zone_deg:
		return current
	if rad_to_deg(current.angle_to(rest_rotation)) <= LANDING_ROTATION_DEG:
		return current
	return current.slerp(rest_rotation, amount).normalized()


func _smoothed_position(amount: float) -> Vector3:
	if _stable_pose.origin.distance_to(_target_pose.origin) <= position_dead_zone_m:
		return _stable_pose.origin
	return _stable_pose.origin.lerp(_target_pose.origin, amount)


func _smoothed_rotation(amount: float) -> Quaternion:
	var stable := _stable_pose.basis.get_rotation_quaternion()
	var target := _target_pose.basis.get_rotation_quaternion()
	if rad_to_deg(stable.angle_to(target)) <= rotation_dead_zone_deg:
		return stable
	return stable.slerp(target, amount).normalized()


func is_ready() -> bool:
	return _ready


func is_reacquiring() -> bool:
	return _reacquiring


## Measurement-only medoid target: it has no dead zone, smoothing or rest-prior bias.
func measurement_target() -> Transform3D:
	return _target_pose


func position_reanchor_ready() -> bool:
	return _position_reanchor_ready


func rotation_reanchor_ready() -> bool:
	return _rotation_reanchor_ready


## Called only after the provider has adopted the stable measurement as its new rest.
func complete_rest_reanchor(position_completed: bool, rotation_completed: bool) -> void:
	if position_completed:
		_position_reanchor_ready = false
	if rotation_completed:
		_rotation_reanchor_ready = false


## Place the avatar at robust rest without discarding genuine measurements collected while hidden.
func anchor_at(pose: Transform3D) -> void:
	_stable_pose = pose
	_target_pose = pose
	_ready = true
	_hold_anchor_once = true
	_reacquiring = false
	_reset_endpoint_state()


## Keep the displayed pose, but ensure reacquisition does not use pre-loss measurements.
func clear_measurement_history() -> void:
	_recent_poses.clear()
	_last_detection_ms = -1
	_has_measurement_target = false
	_reset_endpoint_state()
	if _ready:
		_target_pose = _stable_pose
		_reacquiring = window > 1


## Start completely fresh after startup/re-level rest is confirmed.
func reset() -> void:
	_stable_pose = Transform3D.IDENTITY
	_target_pose = Transform3D.IDENTITY
	_recent_poses.clear()
	_last_detection_ms = -1
	_ready = false
	_hold_anchor_once = false
	_reacquiring = false
	_has_measurement_target = false
	_reset_endpoint_state()


func _reset_endpoint_state() -> void:
	_endpoint_targets.clear()
	_position_reanchor_ready = false
	_rotation_reanchor_ready = false


## Return the actual recent measurement closest to all other recent measurements.
func _medoid() -> Transform3D:
	var best := _recent_poses[0]
	var best_score := INF
	for candidate in _recent_poses:
		var score := 0.0
		for other in _recent_poses:
			score += _pose_distance(candidate, other)
		if score < best_score:
			best_score = score
			best = candidate
	return best


func _pose_distance(a: Transform3D, b: Transform3D) -> float:
	var position_distance_m := a.origin.distance_to(b.origin)
	var rotation_distance_rad := a.basis.get_rotation_quaternion().angle_to(
		b.basis.get_rotation_quaternion()
	)
	return position_distance_m + medoid_rotation_radius_m * rotation_distance_rad


func _smoothing_amount(delta_s: float) -> float:
	if smoothing_time_s <= 0.0:
		return 1.0
	return 1.0 - exp(-maxf(delta_s, 0.0) / smoothing_time_s)
