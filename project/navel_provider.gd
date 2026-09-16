class_name CommonPoseProvider
extends RefCounted
## Reconstructs one common 6DOF pose from any calibrated markers in the newest result.
##
## MARKER SOURCE: this used to hold references to the aruco_patch Node3Ds main_3d.gd wrote poses
## into, and read marker.global_transform off them. The addon publishes markers by ID instead, so
## offsets are keyed by marker id and the poses are handed in as a Dictionary. The calibration file
## follows: its [common] keys are marker ids ("0", "1", "2") rather than node names. Files written
## by the pre-addon build used the node names, and load_from still reads those -- see _offset_key.
##
## SPACE: the poses passed to get_pose() must be WORLD space, because the rig applies the result
## with global_transform. That is ArucoMarkerTracking.get_marker_world_pose(), NOT get_marker_pose()
## -- the latter is play space and would be wrong by the XROrigin3D transform.

signal orientation_settled

# Rest checkpoints selected by the 2026-09-04 grid run (identical winner across both the
# 105 mm-assumed and 617.5 mm-corrected searches): E0 at 30 detections, then every 3.
# Earliest confirmation is detection 42 (4 consecutive stable checks).
var rest_initial_detections := 30
var rest_checkpoint_step := 3
var rest_required_stable_checks := 4
var rest_max_detections := 100
var rest_stable_position_m := 0.0015
var rest_stable_rotation_deg := 1.0

## marker id (int) -> marker-local offset to the common pose.
var _offsets := {}
var _common_pose := Transform3D.IDENTITY
var _has_common_pose := false

var _rest_pose := Transform3D.IDENTITY
var _has_rest_pose := false
var _rest_collecting := true
var _rest_samples: Array[Transform3D] = []
var _rest_last_detection_ms := -1
var _rest_next_checkpoint := 20
var _rest_previous_estimate := Transform3D.IDENTITY
var _rest_has_previous_estimate := false
var _rest_stable_checks := 0
var _rest_reanchor_last_detection_ms := -1


## Convert each visible marker to the common pose, then robustly fuse the estimates.
##
## marker_poses: marker id -> WORLD-space pose, holding one detection's markers and no others.
func get_pose(marker_poses: Dictionary, detection_ms: int = -1) -> Transform3D:
	var estimates: Array[Transform3D] = []
	for id in marker_poses:
		if _offsets.has(id):
			var marker_to_common: Transform3D = _offsets[id]
			estimates.append(marker_poses[id] * marker_to_common)

	if estimates.is_empty():
		return _common_pose

	_common_pose = _floor_lock(_fuse(estimates))
	_has_common_pose = true
	_update_rest(_common_pose, detection_ms)
	return _common_pose


func is_ready() -> bool:
	return _has_common_pose


func rest_pose() -> Transform3D:
	return _rest_pose


func has_rest_pose() -> bool:
	return _has_rest_pose


## Forget only the session rest pose. Permanent marker offsets remain unchanged.
func recalibrate_orientation() -> void:
	_restart_rest_collection()


func _update_rest(measured_pose: Transform3D, detection_ms: int) -> void:
	if _rest_collecting:
		if _collect_rest_sample(measured_pose, detection_ms):
			orientation_settled.emit()
		return

	# Once startup rest exists, only the stabilizer's confirmed stable endpoint may change it.
	# Never educate rest from this raw fused pose.


## Returns true once the checkpoint rule confirms rest or reaches its robust fallback.
func _collect_rest_sample(pose: Transform3D, detection_ms: int) -> bool:
	if not _is_new_detection(pose, detection_ms):
		return false

	_rest_samples.append(pose.orthonormalized())
	var sample_count := _rest_samples.size()
	var at_checkpoint := sample_count >= _rest_next_checkpoint
	var reached_fallback := sample_count >= maxi(rest_max_detections, 1)
	# Fallback is an exact sample budget. It must not wait for the next checkpoint when a future
	# grid result chooses values such as initial=15, step=8, fallback=50.
	if not at_checkpoint and not reached_fallback:
		return false

	var estimate := _robust_estimate(_rest_samples)
	_rest_pose = estimate
	if at_checkpoint:
		_update_stability_count(estimate)
		_rest_previous_estimate = estimate
		_rest_has_previous_estimate = true
		_rest_next_checkpoint += maxi(rest_checkpoint_step, 1)

	var converged := _rest_stable_checks >= rest_required_stable_checks
	if not converged and not reached_fallback:
		return false

	_has_rest_pose = true
	_rest_collecting = false
	_rest_samples.clear()
	if reached_fallback and not converged:
		push_warning(
			"Common pose: rest estimates did not converge; using robust %d-detection fallback."
			% rest_max_detections
		)
	return true


## Accept exactly one rest sample per OpenCV result.
func _is_new_detection(pose: Transform3D, detection_ms: int) -> bool:
	if detection_ms >= 0:
		if detection_ms == _rest_last_detection_ms:
			return false
		_rest_last_detection_ms = detection_ms
		return true

	return _rest_samples.is_empty() or not pose.is_equal_approx(_rest_samples[-1])


func _update_stability_count(estimate: Transform3D) -> void:
	if not _rest_has_previous_estimate:
		return

	var position_change := estimate.origin.distance_to(_rest_previous_estimate.origin)
	var rotation_change_deg := rad_to_deg(
		estimate.basis.get_rotation_quaternion().angle_to(
			_rest_previous_estimate.basis.get_rotation_quaternion()
		)
	)
	if (
		position_change <= rest_stable_position_m
		and rotation_change_deg <= rest_stable_rotation_deg
	):
		_rest_stable_checks += 1
	else:
		_rest_stable_checks = 0


## Adopt the stable measurement-only medoid after the complete target window becomes stationary.
## Position and rotation are updated independently, exactly once per result.
func reanchor_rest_from_stable_target(
		measured_pose: Transform3D,
		detection_ms: int,
		reanchor_position: bool,
		reanchor_rotation: bool
) -> bool:
	if not _has_rest_pose or _rest_collecting:
		return false
	if detection_ms < 0 or detection_ms == _rest_reanchor_last_detection_ms:
		return false
	if not reanchor_position and not reanchor_rotation:
		return false
	_rest_reanchor_last_detection_ms = detection_ms

	var next_position := measured_pose.origin if reanchor_position else _rest_pose.origin
	var next_rotation := (
		measured_pose.basis.get_rotation_quaternion().normalized()
		if reanchor_rotation
		else _rest_pose.basis.get_rotation_quaternion().normalized()
	)
	_rest_pose = Transform3D(Basis(next_rotation), next_position)
	return true


func _restart_rest_collection() -> void:
	_has_rest_pose = false
	_rest_collecting = true
	_rest_samples.clear()
	_rest_last_detection_ms = -1
	_rest_next_checkpoint = maxi(rest_initial_detections, 1)
	_rest_previous_estimate = Transform3D.IDENTITY
	_rest_has_previous_estimate = false
	_rest_stable_checks = 0
	_rest_reanchor_last_detection_ms = -1


func _robust_estimate(poses: Array[Transform3D]) -> Transform3D:
	return Transform3D(Basis(_rotation_medoid(poses)), _median_position(poses))


func _median_position(poses: Array[Transform3D]) -> Vector3:
	var xs: Array[float] = []
	var ys: Array[float] = []
	var zs: Array[float] = []
	for pose in poses:
		xs.append(pose.origin.x)
		ys.append(pose.origin.y)
		zs.append(pose.origin.z)
	xs.sort()
	ys.sort()
	zs.sort()
	return Vector3(_median_value(xs), _median_value(ys), _median_value(zs))


func _median_value(values: Array[float]) -> float:
	var upper := floori(float(values.size()) / 2.0)
	if values.size() % 2 == 1:
		return values[upper]
	return (values[upper - 1] + values[upper]) * 0.5


## Choose the measured rotation with the smallest total angular distance to all others.
func _rotation_medoid(poses: Array[Transform3D]) -> Quaternion:
	var best := poses[0].basis.get_rotation_quaternion().normalized()
	var best_score := INF
	for candidate_pose in poses:
		var candidate := candidate_pose.basis.get_rotation_quaternion().normalized()
		var score := 0.0
		for other_pose in poses:
			score += candidate.angle_to(
				other_pose.basis.get_rotation_quaternion().normalized()
			)
		if score < best_score:
			best_score = score
			best = candidate
	return best


## Enforce the CPR domain rule: the marker/common local Y axis lies along the mannequin on the
## floor and local Z points down. The down sign matches the mannequin child's fixed -90 degree
## import correction so the avatar lies face-up. Only measured horizontal heading is retained.
func _floor_lock(pose: Transform3D) -> Transform3D:
	var up := Vector3.UP
	var body_z := Vector3.DOWN
	var body_y := pose.basis.y.slide(up)
	if body_y.length_squared() < 1.0e-6:
		# A nearly vertical body axis is an unusable ArUco orientation. Keep the last valid heading
		# when possible; before the first valid result, use a deterministic horizontal fallback.
		body_y = _common_pose.basis.y.slide(up) if _has_common_pose else Vector3.FORWARD
	body_y = body_y.normalized()
	var body_x := body_y.cross(body_z).normalized()
	var flat_basis := Basis(body_x, body_y, body_z).orthonormalized()
	# Floor locking constrains orientation only. Marker position remains untouched.
	return Transform3D(flat_basis, pose.origin)


## Fuse the common-pose estimates produced by same-frame markers.
func _fuse(poses: Array[Transform3D]) -> Transform3D:
	# Symmetric sign-aligned mean for every marker count: steadier than median/medoid while all
	# markers are good, at the cost of absorbing 1/3 of a corrupted marker's error. The robust
	# alternative stays in _robust_estimate (still used for rest collection) until the labelled
	# experiment compares mean, robust and agreement-checked mean.
	var position := Vector3.ZERO
	var rotation := Quaternion(0, 0, 0, 0)
	var first_rotation := poses[0].basis.get_rotation_quaternion()

	for pose in poses:
		position += pose.origin
		var pose_rotation := pose.basis.get_rotation_quaternion()
		if first_rotation.dot(pose_rotation) < 0.0:
			pose_rotation = -pose_rotation
		rotation += pose_rotation

	return Transform3D(Basis(rotation.normalized()), position / poses.size())


## The [common] key for one marker id. Spelled as an identifier rather than as the bare number:
## a ConfigFile key starting with a digit is not something this project can verify round-trips
## through Godot's parser without opening the editor, and a calibration that silently fails to
## load falls back to a different one instead of reporting anything.
static func offset_key_for(id: int) -> String:
	return "marker_%d" % id


## Persist marker-local offsets only. World-space rest is relearned every XR session.
## Always written under the id key; the legacy node-name key is read but never produced.
func save_to(path: String) -> void:
	var cfg := ConfigFile.new()
	for id in _offsets:
		cfg.set_value("common", offset_key_for(id), _offsets[id])
	cfg.save(path)


func load_from(path: String, ids: Array) -> bool:
	var cfg := ConfigFile.new()
	if cfg.load(path) != OK:
		return false

	# Validate the complete file before replacing a previously loaded calibration. A partial user
	# file must fail so the caller can fall back to the bundled complete calibration.
	var loaded_offsets := {}
	var required_markers := 0
	for id in ids:
		required_markers += 1
		var key := _offset_key(cfg, id)
		if key.is_empty():
			return false
		var value: Variant = cfg.get_value("common", key)
		if typeof(value) != TYPE_TRANSFORM3D:
			return false
		loaded_offsets[id] = value

	if required_markers == 0 or loaded_offsets.size() != required_markers:
		return false

	_offsets = loaded_offsets
	_restart_rest_collection()
	return true


## The [common] key holding this marker's offset, or "" when the file has none.
##
## The pre-addon build keyed offsets by the scene node's name ("aruco_patch0"), because that is
## what it had. Reading that spelling as well is what keeps a calibration already sitting in
## user:// on a headset -- a measured artefact of a recording session -- from being silently
## discarded on the first run of this build.
func _offset_key(cfg: ConfigFile, id: int) -> String:
	var key := offset_key_for(id)
	if cfg.has_section_key("common", key):
		return key
	var legacy := "aruco_patch%d" % id
	if cfg.has_section_key("common", legacy):
		return legacy
	return ""
