extends Node3D
## Camera-attached practice UI; no claim to measure chest depth or actual breaths.
@export var zone: Node3D
var _title: Label3D
var _counter: Label3D
var _detail: Label3D
var _bar: MeshInstance3D

func _ready() -> void:
	_panel(Vector2(0.36, 0.20), Vector3.ZERO, Color(0.015, 0.045, 0.06, 0.92))
	_title = _label(Vector3(0, 0.068, 0.002), 28)
	_counter = _label(Vector3(0, 0.022, 0.002), 52)
	_detail = _label(Vector3(0, -0.066, 0.002), 23)
	_panel(Vector2(0.29, 0.009), Vector3(0, -0.026, 0.002), Color(0.08, 0.18, 0.21))
	_bar = _panel(Vector2(0.29, 0.009), Vector3(0, -0.026, 0.003), Color(0.2, 0.8, 0.95))
	visible = false

func _label(at: Vector3, font: int) -> Label3D:
	var label := Label3D.new()
	label.position = at
	label.font_size = font
	label.pixel_size = 0.00045
	label.outline_size = 4
	label.no_depth_test = true
	label.modulate = Color(0.88, 0.98, 1.0)
	add_child(label)
	return label

func _panel(size_value: Vector2, at: Vector3, colour: Color) -> MeshInstance3D:
	var panel := MeshInstance3D.new()
	var mesh := QuadMesh.new()
	mesh.size = size_value
	panel.mesh = mesh
	panel.position = at
	panel.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.no_depth_test = true
	material.albedo_color = colour
	panel.material_override = material
	add_child(panel)
	return panel

func _process(_delta: float) -> void:
	visible = zone != null and zone.is_visible_in_tree() and not zone.get("_paused")
	if not visible:
		return
	var session = zone.get("motion_session")
	var fraction := 0.0
	if session.phase == "breathing":
		_title.text = "GIVE 2 BREATHS"
		_counter.text = "%.1f s" % session.breathing_remaining_s
		_detail.text = "30/30 compressions\nPractice pause"
		fraction = session.breathing_remaining_s / maxf(0.1, session.breathing_duration_s)
	else:
		if not session.active:
			_title.text = "LISTEN TO BEEP"
			_counter.text = ""
			_detail.text = "Waiting for first\ncompression..."
			fraction = 0.0
		else:
			_title.text = "COMPRESS"
			_counter.text = "%d / 30" % session.count
			if session.travel_m < 0.050:
				_detail.text = "Press deeper: %.1f cm\n%.0f BPM" % [session.travel_m * 100.0, session.target_bpm]
			else:
				_detail.text = "Good depth: %.1f cm\n%.0f BPM" % [session.travel_m * 100.0, session.target_bpm]
			fraction = session.travel_m / 0.08
	_bar.scale.x = maxf(0.001, clampf(fraction, 0.0, 1.0))
	_bar.position.x = -0.145 + 0.145 * _bar.scale.x
