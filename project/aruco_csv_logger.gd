extends Node
## Save each ArUco detection result as one labelled CSV row.
##
## MARKER SOURCE. This used to reach into main_3d.gd's private detection slots -- it took that
## script's _detect_mutex, copied _result_markers and _result_cam_xform under it, and de-duplicated
## by hashing the result because Godot renders faster than OpenCV detects. None of that exists any
## more: the opencv_aruco addon emits detection_applied(image, corners, poses, head_pose) exactly
## once per finished detection, on the main thread, so the row is written from the signal and both
## the lock and the hash are gone.
##
## THE CSV COLUMNS ARE UNCHANGED ON PURPOSE, because the analysis scripts (tune_filter.py and the
## rest of the recording-instructions toolchain) parse them. That takes one conversion:
##
##   camera_*   the head pose at the frame's CAPTURE time. Was xr_camera.global_transform sampled
##              by hand from a pose history; is now the addon's head_pose, which comes from
##              xrLocateSpace at the frame's own timestamp -- the same quantity, measured better.
##              Play space rather than world space, i.e. identical while the XROrigin3D sits at
##              identity (it does; see cpr_trainer.tscn).
##   <marker>_* the marker pose in CAMERA space, as before. The addon hands out poses with
##              head_pose * lens_pose already applied, so the camera-space value the old pipeline
##              logged is recovered exactly by undoing the head pose: head_pose^-1 * play_pose
##              leaves lens_pose * marker_optical, which is what the old C++ returned.
##
## Logging the addon's poses raw instead would silently change what every existing CSV column
## means, and nothing downstream would report it.

const MARKER_IDS := [0, 1, 2]
const MARKER_NAMES := ["common", "chest", "navel"]
const TEST_TYPES := ["calibration", "stationary", "headmotion", "moving", "hiding", "compressions"]
# Where the head was standing while the markers stayed put. A marker's pose error is systematic
# per VIEWPOINT, so labelling the viewpoint is what lets the analysis ask whether the offset
# learned from the left of the mannequin agrees with the one learned from the right. Without
# these labels a walk is just an undifferentiated pile of rows.
const CALIBRATION_PHASES := [
	"near", "far", "left_side", "right_side", "crouch", "stand_tall"
]
const STATIONARY_PHASES := [
	"stationary_front", "stationary_left_view", "stationary_right_view",
	"stationary_high_view", "stationary_low_view", "stationary_far_view"
]
# How fast the head was moving. Capture latency displaces a marker in proportion to head speed,
# so a recording that never changes speed cannot separate latency from anything else.
const HEADMOTION_PHASES := ["still", "slow", "medium", "fast"]
const MOVING_PHASES := [
	"stationary_A", "moving_A_to_B", "stationary_B",
	"moving_B_to_A", "stationary_A_return"
]
const HIDING_PHASES := [
	"all_visible", "common_hidden", "all_visible", "chest_hidden",
	"all_visible", "navel_hidden", "all_visible"
]
# Real chest compressions mix occlusion, marker flicker and mannequin rocking. The still phases
# on both sides give the analysis a same-session baseline to compare the compression rows against.
const COMPRESSION_PHASES := ["still_before", "compressions", "still_after"]
const POSE_SUFFIXES := ["x", "y", "z", "qx", "qy", "qz", "qw"]

## The addon node whose detections are logged.
@export var marker_tracking: ArucoMarkerTracking
@export var right_controller: XRController3D
@export var left_controller: XRController3D
# Typed as Node, not Label, so this accepts a Label3D. A CanvasLayer Label does not render
# inside the headset on this setup - only on the mirrored view - which left the recording
# state invisible while wearing it. A Label3D parented to the camera always renders in VR.
# Both node types expose .text, so _update_status works either way.
@export var status_label: Node

var _file: FileAccess
var _recording := false
var _sample_id := 0
var _test_index := 0
var _phase_index := 0
var _recording_start_ms := 0


func _ready() -> void:
	# Right A controls the recording; left X marks the next experiment phase.
	if right_controller != null:
		right_controller.button_pressed.connect(_on_right_button)
	if left_controller != null:
		left_controller.button_pressed.connect(_on_left_button)
	if marker_tracking != null:
		marker_tracking.detection_applied.connect(_on_detection_applied)
	else:
		push_error("ArUco logger: no ArucoMarkerTracking assigned; nothing will be recorded.")
	_update_status()


func _on_right_button(button: StringName) -> void:
	if button != &"ax_button":
		return
	if _recording:
		_stop_recording()
	else:
		_start_recording()


func _on_left_button(button: StringName) -> void:
	# While stopped X selects the test; while recording X marks the next phase. Y remains an
	# alternative test selector, but the experiment never depends on it.
	if button == &"ax_button":
		if _recording:
			_phase_index = (_phase_index + 1) % _current_phases().size()
			print("ArUco phase: ", _current_phase())
		else:
			_select_next_test()
		_update_status()
	elif button == &"by_button":
		_select_next_test()
		_update_status()


func _select_next_test() -> void:
	_test_index = (_test_index + 1) % TEST_TYPES.size()
	_phase_index = 0
	print("ArUco test type: ", _current_test())


## One finished detection, one row. A detection that found no markers still gets a row, with the
## marker columns empty -- that is what the `hiding` test is made of.
func _on_detection_applied(
		_image: Image,
		_corners: Dictionary,
		poses: Dictionary,
		head_pose: Transform3D
) -> void:
	if not _recording:
		return
	_write_row(poses, head_pose)


func _start_recording() -> void:
	# The timestamp prevents a later recording from overwriting an earlier one.
	var path := "user://aruco_%s_%d.csv" % [
		_current_test(), int(Time.get_unix_time_from_system())
	]
	_file = FileAccess.open(path, FileAccess.WRITE)
	if _file == null:
		push_error("Could not create %s" % path)
		return

	_file.store_line(",".join(_make_header()))
	_file.flush()
	_sample_id = 0
	# Always begin at phase 1, whatever X was pressed beforehand while stopped.
	_phase_index = 0
	_recording_start_ms = Time.get_ticks_msec()
	_recording = true
	print("ArUco logger started: ", path, "  phase=", _current_phase())
	_update_status()


func _stop_recording() -> void:
	_recording = false
	_file.flush()
	_file.close()
	_file = null
	print("ArUco logger stopped after ", _sample_id, " samples.")
	# The test type deliberately does NOT advance here. It used to, which meant the name a file
	# got depended on how many recordings preceded it rather than on what was in it - every file
	# from a session came out mislabelled. Y chooses the type; stopping only rewinds the phase.
	_phase_index = 0
	_update_status()


func _write_row(poses: Dictionary, head_pose: Transform3D) -> void:
	var row := PackedStringArray([
		str(_sample_id),
		str(Time.get_ticks_msec()),
		str(Time.get_ticks_msec() - _recording_start_ms),
		_current_test(),
		_current_phase(),
	])
	_add_pose(row, head_pose)
	# Undo the head pose to get back to the camera-space columns the format has always carried.
	# Inverted once per row rather than once per marker: it is the same matrix for all three.
	var to_camera := head_pose.affine_inverse()
	for marker_id in MARKER_IDS:
		# has() rather than get(): a missing marker means "not seen in this frame" and must leave
		# its columns empty, and a Transform3D cannot be null to signal that on its own.
		if poses.has(marker_id):
			_add_pose(row, to_camera * (poses[marker_id] as Transform3D))
		else:
			_add_pose(row, null)

	_file.store_line(",".join(row))
	_file.flush()
	_sample_id += 1


# Adds one Transform3D pose to the current CSV row.
# The pose position is saved as x, y, and z.
# The pose rotation is converted to a quaternion and saved as qx, qy, qz, and qw.
# Each value is converted to text with 9 digits after the decimal point.
# A null pose means the marker was not seen, so the same columns are left empty
# and the row still matches the header.
func _add_pose(row: PackedStringArray, pose: Variant) -> void:
	if pose == null:
		for _unused in POSE_SUFFIXES.size():
			row.append("")
		return

	var transform: Transform3D = pose
	var q := transform.basis.get_rotation_quaternion()
	for value in [
		transform.origin.x, transform.origin.y, transform.origin.z,
		q.x, q.y, q.z, q.w,
	]:
		row.append("%.9f" % value)


func _make_header() -> PackedStringArray:
	var header := PackedStringArray([
		"sample_id", "logger_ms", "recording_ms", "test_type", "phase_label"
	])
	_add_pose_names(header, "camera")
	for marker_name in MARKER_NAMES:
		_add_pose_names(header, marker_name)
	return header


func _add_pose_names(header: PackedStringArray, name: String) -> void:
	for suffix in POSE_SUFFIXES:
		header.append(name + "_" + suffix)


func _current_test() -> String:
	return TEST_TYPES[_test_index]


func _current_phases() -> Array:
	match _current_test():
		"calibration":
			return CALIBRATION_PHASES
		"headmotion":
			return HEADMOTION_PHASES
		"moving":
			return MOVING_PHASES
		"hiding":
			return HIDING_PHASES
		"compressions":
			return COMPRESSION_PHASES
		_:
			return STATIONARY_PHASES


func _current_phase() -> String:
	return _current_phases()[_phase_index]


func _update_status() -> void:
	if status_label == null:
		return
	var state := "RECORDING" if _recording else "STOPPED"
	# Which phase number it is matters while recording: the label alone does not say whether the
	# press registered when two phases in a row carry the same name.
	var controls := "A: start   X: select test" if not _recording else "A: stop   X: next phase"
	status_label.text = "%s - %s\nPhase %d/%d: %s\n%s" % [
		state, _current_test(),
		_phase_index + 1, _current_phases().size(), _current_phase(), controls
	]


func _exit_tree() -> void:
	if _recording:
		_stop_recording()
