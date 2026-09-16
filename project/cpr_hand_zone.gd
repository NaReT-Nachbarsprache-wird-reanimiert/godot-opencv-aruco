extends Area3D
## Green visual placement guide by default. Experimental heel grading is opt-in.
## Uses tracked hand motion to start a sensor-free practice cycle after two strokes.

signal placement_changed(correct: bool)
signal cpr_started
signal placement_restarted

## Green marks the intended target; it does not indicate detected correct placement.
@export var guide_only := true
@export var xr_origin: XROrigin3D
@export var start_controller: XRController3D
@export var start_button: StringName = &"by_button"
@export var left_tracker: StringName = &"/user/hand_tracker/left"
@export var right_tracker: StringName = &"/user/hand_tracker/right"

@export_group("Heel estimate")
## Fraction from wrist centre toward palm centre; no heel joint exists in OpenXR.
@export_range(0.0, 1.0, 0.05) var heel_wrist_to_palm_fraction := 0.3
## Approximate joint-centre to skin offset toward the palm-facing side, in metres.
@export_range(0.0, 0.025, 0.001) var heel_surface_offset_m := 0.008
@export_group("Stack tolerances")
@export var stack_min_height_m := 0.008
@export var stack_max_height_m := 0.080
@export var stack_lateral_tolerance_m := 0.035
@export_range(0.0, 80.0, 1.0) var max_palm_tilt_degrees := 50.0

@export_group("Hand-motion practice")
@export var enable_motion_practice := true
@export_range(100.0, 120.0, 5.0) var target_bpm := 110.0
## Configurable practice pause, not detection of breaths.
@export_range(2.0, 9.0, 0.5) var breathing_duration_s := 5.0

var left_hand_inside := false
var right_hand_inside := false
var correct_placement: bool:
	get:
		return _placement_correct
## The hand whose estimated heel is in the chest contact slab, or empty when incorrect.
var lower_hand: StringName = &""
var _placement_correct := false
var is_cpr_started := false
var _paused := false
@onready var _shape: CollisionShape3D = $CollisionShape3D
@onready var _highlight: MeshInstance3D = $Highlight
@onready var _instruction: Label3D = $Instruction
var _material: StandardMaterial3D

var motion_session = preload("res://cpr_motion_session.gd").new()
var _motion_tracker: StringName = &""
var _last_chest_pose := Transform3D.IDENTITY
var _have_chest_pose := false
var hand_tracking_status := "Show your hands"
var _motion_reasons := {}
var _last_tracking_status := ""
var _metronome: AudioStreamPlayer


func _ready() -> void:
	_material = StandardMaterial3D.new()
	_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	if not guide_only:
		_highlight.material_override = _material
	if start_controller != null:
		start_controller.button_pressed.connect(_on_start_button)
	_metronome = AudioStreamPlayer.new()
	_metronome.set_script(preload("res://cpr_metronome.gd"))
	add_child(_metronome)
	motion_session.beat_requested.connect(_metronome.play.bind(0.0))
	_update_feedback()


func _process(delta: float) -> void:
	var was_correct := correct_placement
	var available := not _paused and is_visible_in_tree()
	var read_hands := available and not guide_only
	var left := _read_hand(left_tracker) if read_hands else {}
	var right := _read_hand(right_tracker) if read_hands else {}
	left_hand_inside = not guide_only and _heel_on_target(left)
	right_hand_inside = not guide_only and _heel_on_target(right)
	lower_hand = &""
	if left_hand_inside and _is_stacked(left, right):
		lower_hand = &"left"
	elif right_hand_inside and _is_stacked(right, left):
		lower_hand = &"right"
	_placement_correct = lower_hand != &""
	if correct_placement != was_correct:
		placement_changed.emit(correct_placement)
	if enable_motion_practice:
		var motion_left := _read_motion_hand(left_tracker) if available else {}
		var motion_right := _read_motion_hand(right_tracker) if available else {}
		_update_hand_motion(delta, available, motion_left, motion_right)
	_update_feedback()


func _update_hand_motion(delta: float, available: bool, left: Dictionary, right: Dictionary) -> void:
	motion_session.target_bpm = target_bpm
	motion_session.breathing_duration_s = breathing_duration_s
	var sample := {}
	var chosen: StringName = &""
	# Keep the same hand through a stroke. Prefer right if both first become available;
	# the visible upper hand is sufficient for motion estimation, not placement approval.
	if _motion_tracker == left_tracker and _motion_hand_usable(left):
		sample = left
		chosen = left_tracker
	elif _motion_tracker == right_tracker and _motion_hand_usable(right):
		sample = right
		chosen = right_tracker
	elif _motion_hand_usable(right):
		sample = right
		chosen = right_tracker
	elif _motion_hand_usable(left):
		sample = left
		chosen = left_tracker
	if chosen != _motion_tracker:
		motion_session.invalidate_tracking()
		_motion_tracker = chosen
	var chest := _shape.global_transform
	var units := _units_per_metre()
	var jumped := _have_chest_pose and (
		chest.origin.distance_to(_last_chest_pose.origin) > 0.03 * units
		or chest.basis.orthonormalized().get_rotation_quaternion().angle_to(
			_last_chest_pose.basis.orthonormalized().get_rotation_quaternion()) > deg_to_rad(5.0))
	_last_chest_pose = chest
	_have_chest_pose = available
	var valid := available and not jumped and not sample.is_empty()
	var height := 0.0
	if valid:
		height = (sample.point - chest.origin).dot(chest.basis.y.normalized()) / units
	if not sample.is_empty():
		hand_tracking_status = "Hand tracked"
	elif not left.is_empty() or not right.is_empty():
		hand_tracking_status = "Move your hand over the ring"
	elif _motion_reasons.values().has("controller"):
		hand_tracking_status = "Put controllers down to use hands"
	elif _motion_reasons.values().has("no_tracker"):
		hand_tracking_status = "Enable hand tracking on Quest"
	else:
		hand_tracking_status = "Keep your hands in view"
	if available and hand_tracking_status != _last_tracking_status:
		_last_tracking_status = hand_tracking_status
		print("CPR hands: ", hand_tracking_status, " | ", _motion_reasons)
	motion_session.update(delta, valid, height, not available)
	if motion_session.active and not is_cpr_started:
		is_cpr_started = true
		cpr_started.emit()
	if not available:
		_metronome.stop()


func _units_per_metre() -> float:
	if xr_origin == null:
		return 1.0
	return maxf(0.001, XRServer.world_scale * xr_origin.global_basis.get_scale().length() / sqrt(3.0))


func _motion_hand_usable(hand: Dictionary) -> bool:
	if hand.is_empty():
		return false
	var offset: Vector3 = hand.point - _shape.global_position
	var units := _units_per_metre()
	var height := offset.dot(_shape.global_basis.y.normalized()) / units
	var sideways := offset.dot(_shape.global_basis.x.normalized()) / units
	var lengthwise := offset.dot(_shape.global_basis.z.normalized()) / units
	return absf(sideways) < 0.08 and absf(lengthwise) < 0.08 and height > -0.12 and height < 0.15

## Motion needs only an actively tracked palm position, not a wrist or orientation.
## Keep the stricter heel/orientation checks exclusively in the experimental placement grader.
func _read_motion_hand(tracker_name: StringName) -> Dictionary:
	_motion_reasons[tracker_name] = "no_tracker"
	if xr_origin == null:
		return {}
	var hand := XRServer.get_tracker(tracker_name) as XRHandTracker
	if hand == null:
		return {}
	if hand.hand_tracking_source == XRHandTracker.HAND_TRACKING_SOURCE_CONTROLLER:
		_motion_reasons[tracker_name] = "controller"
		return {}
	_motion_reasons[tracker_name] = "not_tracked"
	if not hand.has_tracking_data or hand.hand_tracking_source == XRHandTracker.HAND_TRACKING_SOURCE_NOT_TRACKED:
		return {}
	var flags := hand.get_hand_joint_flags(XRHandTracker.HAND_JOINT_PALM)
	var required := XRHandTracker.HAND_JOINT_FLAG_POSITION_VALID | XRHandTracker.HAND_JOINT_FLAG_POSITION_TRACKED
	if (flags & required) != required:
		_motion_reasons[tracker_name] = "palm_not_tracked (flags=%d)" % flags
		return {}
	var palm := hand.get_hand_joint_transform(XRHandTracker.HAND_JOINT_PALM)
	if not palm.origin.is_finite():
		return {}
	var pose := XRPose.new()
	pose.transform = Transform3D(Basis.IDENTITY, palm.origin)
	var position_world := xr_origin.global_transform * pose.get_adjusted_transform().origin
	_motion_reasons[tracker_name] = "tracked"
	return {"point": position_world}


func _read_hand(tracker_name: StringName) -> Dictionary:
	if xr_origin == null:
		return {}
	var hand := XRServer.get_tracker(tracker_name) as XRHandTracker
	if hand == null or not hand.has_tracking_data:
		return {}
	if hand.hand_tracking_source in [XRHandTracker.HAND_TRACKING_SOURCE_CONTROLLER, XRHandTracker.HAND_TRACKING_SOURCE_NOT_TRACKED]:
		return {}
	var position_flags := XRHandTracker.HAND_JOINT_FLAG_POSITION_VALID | XRHandTracker.HAND_JOINT_FLAG_POSITION_TRACKED
	var palm_flags := position_flags | XRHandTracker.HAND_JOINT_FLAG_ORIENTATION_VALID | XRHandTracker.HAND_JOINT_FLAG_ORIENTATION_TRACKED
	if (hand.get_hand_joint_flags(XRHandTracker.HAND_JOINT_PALM) & palm_flags) != palm_flags:
		return {}
	if (hand.get_hand_joint_flags(XRHandTracker.HAND_JOINT_WRIST) & position_flags) != position_flags:
		return {}
	var palm := hand.get_hand_joint_transform(XRHandTracker.HAND_JOINT_PALM)
	var wrist := hand.get_hand_joint_transform(XRHandTracker.HAND_JOINT_WRIST)
	if not palm.is_finite() or not wrist.origin.is_finite() or absf(palm.basis.determinant()) < 0.001:
		return {}
	var wrist_to_palm := wrist.origin.distance_to(palm.origin)
	if wrist_to_palm < 0.010 or wrist_to_palm > 0.150:
		return {}
	# Godot's OpenXR Humanoid conversion makes -Z face out the back of the hand:
	# +Z therefore points toward the palm skin/contact surface (both left and right).
	var heel := wrist.origin.lerp(palm.origin, heel_wrist_to_palm_fraction)
	heel += palm.basis.z.normalized() * heel_surface_offset_m
	var pose := XRPose.new()
	pose.transform = Transform3D(palm.basis.orthonormalized(), heel)
	# Apply the same reference frame and world scale as XRNode3D, then the scene origin.
	var world := xr_origin.global_transform * pose.get_adjusted_transform()
	return {"heel": world.origin, "palm_normal": world.basis.z.normalized()}


func _heel_on_target(hand: Dictionary) -> bool:
	return not hand.is_empty() and contains_world_point(hand.heel) and _faces_chest(hand)


func _faces_chest(hand: Dictionary) -> bool:
	var chest_normal := _shape.global_basis.y.normalized()
	return hand.palm_normal.dot(-chest_normal) >= cos(deg_to_rad(max_palm_tilt_degrees))


func _is_stacked(lower: Dictionary, upper: Dictionary) -> bool:
	if upper.is_empty() or not _faces_chest(upper):
		return false
	var chest_normal := _shape.global_basis.y.normalized()
	var separation: Vector3 = upper.heel - lower.heel
	var height := separation.dot(chest_normal)
	# Distances are specified in physical metres, independent of the mannequin's 0.77 scale.
	var units_per_metre := XRServer.world_scale * xr_origin.global_basis.get_scale().length() / sqrt(3.0)
	if height < stack_min_height_m * units_per_metre or height > stack_max_height_m * units_per_metre:
		return false
	var lateral := (separation - chest_normal * height).length()
	return lateral <= stack_lateral_tolerance_m * units_per_metre


func contains_world_point(world_position: Vector3) -> bool:
	var box := _shape.shape as BoxShape3D
	if box == null or _shape.disabled or not world_position.is_finite():
		return false
	var local := _shape.to_local(world_position)
	var half := box.size * 0.5
	return absf(local.x) <= half.x and absf(local.y) <= half.y and absf(local.z) <= half.z


## Call from the explicit Start action or a future compression detector.
func start_cpr() -> void:
	if is_cpr_started or _paused or not is_visible_in_tree():
		return
	is_cpr_started = true
	motion_session.start()
	_motion_tracker = &""
	_clear_placement()
	_update_feedback()
	cpr_started.emit()

func reset_placement() -> void:
	is_cpr_started = false
	motion_session.reset()
	_motion_tracker = &""
	_have_chest_pose = false
	if _metronome != null:
		_metronome.stop()
	_clear_placement()
	_update_feedback()
	placement_restarted.emit()

func _clear_placement() -> void:
	var was_correct := correct_placement
	left_hand_inside = false
	right_hand_inside = false
	_placement_correct = false
	lower_hand = &""
	if was_correct:
		placement_changed.emit(false)


func _update_feedback() -> void:
	if _highlight == null or _material == null:
		return
	_highlight.visible = not is_cpr_started and not _paused
	_instruction.visible = _highlight.visible
	if not guide_only:
		_material.albedo_color = Color(0.1, 0.9, 0.25, 0.75) if correct_placement else Color(1.0, 0.35, 0.05, 0.75)


func _on_start_button(button: StringName) -> void:
	if button == start_button:
		if is_cpr_started:
			reset_placement()
		else:
			start_cpr()


func _unhandled_key_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_C:
		_on_start_button(start_button)
		get_viewport().set_input_as_handled()


func _notification(what: int) -> void:
	if what == NOTIFICATION_APPLICATION_PAUSED:
		_paused = true
		motion_session.invalidate_tracking()
		if _metronome != null:
			_metronome.stop()
		_clear_placement()
		_update_feedback()
	elif what == NOTIFICATION_APPLICATION_RESUMED:
		_paused = false
		motion_session.invalidate_tracking()
		_have_chest_pose = false
		_clear_placement()
		_update_feedback()
