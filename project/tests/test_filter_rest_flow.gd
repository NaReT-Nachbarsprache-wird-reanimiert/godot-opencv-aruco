extends SceneTree


class MockFloorProvider extends Node:
	var height := 0.0

	func has_floor() -> bool:
		return true

	func floor_height_world() -> float:
		return height


func _init() -> void:
	# SceneTree._init runs before Node3D global transforms are available. Defer the suite by one
	# loop turn so the floor-contact geometry test can use real scene-tree transforms.
	call_deferred("_run_tests")


func _run_tests() -> void:
	_test_floor_lock_discards_marker_tilt()
	_test_avatar_lowest_point_snaps_to_quest_floor()
	_test_prior_still_works_inside_rest_dead_zone()
	_test_relocation_threshold_is_separate_from_dead_zone()
	_test_slow_movement_is_not_a_stable_endpoint()
	_test_stable_target_reanchors_once()
	_test_display_lands_after_reanchor()
	print("filter/rest flow tests: PASS")
	quit()


func _test_avatar_lowest_point_snaps_to_quest_floor() -> void:
	var rig_script := preload("res://avatar_rig_navel.gd")
	var rig: Node3D = rig_script.new()
	var avatar := Node3D.new()
	var mesh_instance := MeshInstance3D.new()
	var box := BoxMesh.new()
	box.size = Vector3(1.0, 0.2, 2.0)
	mesh_instance.mesh = box
	rig.add_child(avatar)
	avatar.add_child(mesh_instance)
	rig.target = avatar
	var floor_provider := MockFloorProvider.new()
	floor_provider.height = 0.0
	rig.add_child(floor_provider)
	rig.floor_provider = floor_provider
	# The rig takes its markers from the addon by ID now, so there are no aruco_patch stand-ins to
	# build. It still needs a tracking node to subscribe to: an ArucoMarkerTracking that never
	# enters the tree builds no OpenCVProcessor and starts no camera, so it costs nothing here and
	# keeps the rig's _ready off its "no marker source" error path.
	var tracking := ArucoMarkerTracking.new()
	rig.marker_tracking = tracking
	# Enter the tree so Node3D global transforms are valid, but do not run the tracking loop.
	rig.process_mode = Node.PROCESS_MODE_DISABLED
	root.add_child(rig)

	# Start 35 cm below the floor. A 20 cm-tall mesh has its bottom 10 cm below its rig origin.
	rig._apply_filtered_pose(Transform3D(Basis.IDENTITY, Vector3(4.0, -0.35, 5.0)))
	assert(absf(rig._lowest_mesh_world_y(avatar)) < 1.0e-5)
	assert(is_equal_approx(rig.global_position.x, 4.0))
	assert(is_equal_approx(rig.global_position.z, 5.0))
	rig.free()
	tracking.free()


func _test_floor_lock_discards_marker_tilt() -> void:
	var provider := CommonPoseProvider.new()
	var tilted := Transform3D(
		Basis(Vector3(1.0, 0.0, 0.0), deg_to_rad(37.0)),
		Vector3(1.0, 2.0, 3.0)
	)
	var flat := provider._floor_lock(tilted)
	assert(flat.origin.is_equal_approx(tilted.origin))
	assert(absf(flat.basis.y.dot(Vector3.UP)) < 1.0e-5)
	assert(flat.basis.z.is_equal_approx(Vector3.DOWN))
	assert(flat.basis.determinant() > 0.999)


func _test_prior_still_works_inside_rest_dead_zone() -> void:
	var filter := SimplePoseStabilizer.new()
	filter.configure(7, 0.006, 0.5, 0.8, 0.1)
	var rest := Transform3D.IDENTITY
	var near_rest := Transform3D(Basis.IDENTITY, Vector3(0.004, 0.0, 0.0))
	filter.anchor_at(near_rest)
	var output := near_rest
	for frame in 30:
		output = filter.update(near_rest, 1.0 / 72.0, 1, rest, true)
	assert(output.origin.x < near_rest.origin.x)


func _test_relocation_threshold_is_separate_from_dead_zone() -> void:
	var filter := SimplePoseStabilizer.new()
	filter.configure(7, 0.006, 0.5, 0.8, 8.0, 0.002, 0.5, 7, 0.020, 6.0)
	var rest := Transform3D.IDENTITY
	# Twelve millimetres exceeds the display dead zone, but must not change remembered rest.
	var moved := Transform3D(Basis.IDENTITY, Vector3(0.012, 0.0, 0.0))
	filter.anchor_at(rest)
	for detection in 30:
		filter.update(moved, 1.0 / 72.0, detection, rest, true)
	assert(not filter.position_reanchor_ready())

	# A stable 30 mm endpoint exceeds the independent 20 mm relocation threshold.
	moved = Transform3D(Basis.IDENTITY, Vector3(0.030, 0.0, 0.0))
	for detection in range(30, 60):
		filter.update(moved, 1.0 / 72.0, detection, rest, true)
	assert(filter.position_reanchor_ready())
	assert(not filter.rotation_reanchor_ready())

	var rotation_filter := SimplePoseStabilizer.new()
	rotation_filter.configure(7, 0.006, 0.5, 0.8, 8.0, 0.002, 0.5, 7, 0.020, 6.0)
	rotation_filter.anchor_at(rest)
	var rotated := Transform3D(Basis(Vector3.UP, deg_to_rad(4.0)), Vector3.ZERO)
	for detection in 30:
		rotation_filter.update(rotated, 1.0 / 72.0, detection, rest, true)
	assert(not rotation_filter.rotation_reanchor_ready())
	rotated = Transform3D(Basis(Vector3.UP, deg_to_rad(8.0)), Vector3.ZERO)
	for detection in range(30, 60):
		rotation_filter.update(rotated, 1.0 / 72.0, detection, rest, true)
	assert(rotation_filter.rotation_reanchor_ready())


func _test_slow_movement_is_not_a_stable_endpoint() -> void:
	var filter := SimplePoseStabilizer.new()
	filter.configure(7, 0.006, 0.5, 0.8, 8.0, 0.002, 0.5, 7)
	var rest := Transform3D.IDENTITY
	filter.anchor_at(rest)
	for detection in 30:
		var moving := Transform3D(
			Basis.IDENTITY,
			Vector3(float(detection + 1) * 0.001, 0.0, 0.0)
		)
		filter.update(moving, 1.0 / 72.0, detection, rest, true)
	assert(not filter.position_reanchor_ready())


func _test_stable_target_reanchors_once() -> void:
	var filter := SimplePoseStabilizer.new()
	filter.configure(7, 0.006, 0.5, 0.8, 8.0, 0.002, 0.5, 7)
	var old_rest := Transform3D.IDENTITY
	var moved := Transform3D(Basis.IDENTITY, Vector3(0.020, 0.0, 0.0))
	filter.anchor_at(old_rest)
	for detection in 30:
		filter.update(moved, 1.0 / 72.0, detection, old_rest, true)
	assert(filter.position_reanchor_ready())

	var provider := CommonPoseProvider.new()
	provider._has_rest_pose = true
	provider._rest_collecting = false
	provider._rest_pose = old_rest
	assert(provider.reanchor_rest_from_stable_target(moved, 42, true, false))
	filter.complete_rest_reanchor(true, false)
	assert(is_equal_approx(provider.rest_pose().origin.x, 0.020))
	assert(not filter.position_reanchor_ready())

	# The same OpenCV result cannot re-anchor twice.
	var duplicate := Transform3D(Basis.IDENTITY, Vector3(0.200, 0.0, 0.0))
	assert(not provider.reanchor_rest_from_stable_target(duplicate, 42, true, false))
	assert(is_equal_approx(provider.rest_pose().origin.x, 0.020))


func _test_display_lands_after_reanchor() -> void:
	var filter := SimplePoseStabilizer.new()
	filter.configure(7, 0.006, 0.5, 0.8, 8.0)
	# After a re-anchor rest equals the measurement, but the display sits 5 mm short - inside the
	# dead zone. Without landing, only the 8 s prior closes this and 3.9 mm would remain after 2 s.
	var moved := Transform3D(Basis.IDENTITY, Vector3(0.050, 0.0, 0.0))
	var short_of_target := Transform3D(Basis.IDENTITY, Vector3(0.045, 0.0, 0.0))
	filter.anchor_at(short_of_target)
	var output := short_of_target
	for detection in 144:
		output = filter.update(moved, 1.0 / 72.0, detection, moved, true)
	assert(output.origin.distance_to(moved.origin) < 0.0015)
