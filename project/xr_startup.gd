extends XROrigin3D

# Minimal OpenXR startup: find the interface, initialise it, and switch the
# viewport to XR. Works on both desktop (no headset -> flat fallback) and Quest.
#
# Extends XROrigin3D rather than Node because the CPR avatar rig needs the physical floor height
# and this node IS the floor: with the LOCAL_FLOOR reference space (xr/openxr/reference_space=2 in
# project.godot) OpenXR puts the origin on the Quest's calibrated floor plane. has_floor() /
# floor_height_world() below are the whole interface the rig uses for that, so it never has to know
# which reference space is active.
#
# This does NOT initialise OpenXR -- xr/openxr/enabled=true in project.godot means the engine has
# already done that before the main scene loads.

var xr_interface: XRInterface


func _ready() -> void:
	print("available interfaces:",XRServer.get_interfaces())
	xr_interface = XRServer.find_interface("OpenXR")
	if xr_interface and xr_interface.is_initialized():
		print("OpenXR initialised successfully; requested reference space: LOCAL_FLOOR")
		print("OpenXR play-area mode: ", xr_interface.get_play_area_mode(),
			" (ROOMSCALE means a floor-based space is active)")
		xr_interface.play_area_changed.connect(_on_play_area_changed)
		# The XR compositor drives frame pacing; disable the desktop v-sync. (dont wait
		DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
		get_viewport().use_xr = true

		#OpenXRMetaEnvironmentDepthExtension.start_environment_depth()
		# optional: keep your own hands from occluding the patches
		#OpenXRMetaEnvironmentDepthExtension.set_hand_removal_enabled(true)


		_enable_passthrough()
	else:
		print("OpenXR not initialised - running in flat (non-XR) mode")


func _on_play_area_changed(mode: XRInterface.PlayAreaMode) -> void:
	print("OpenXR play-area changed: ", mode)


## With the LOCAL_FLOOR reference space this origin sits on the physical floor, so its world
## height is the Quest floor height. In flat desktop mode there is no floor estimate.
func has_floor() -> bool:
	return xr_interface != null and xr_interface.is_initialized()


func floor_height_world() -> float:
	return global_position.y


# Meta passthrough (AR): composite the rendered scene over the real world by switching the
# XR environment blend mode to alpha-blend and clearing the viewport to transparent. Wherever
# the scene draws nothing, the real-world camera shows through; opaque meshes render on top.
func _enable_passthrough() -> void:
	var modes := xr_interface.get_supported_environment_blend_modes()
	if XRInterface.XR_ENV_BLEND_MODE_ALPHA_BLEND in modes:
		xr_interface.environment_blend_mode = XRInterface.XR_ENV_BLEND_MODE_ALPHA_BLEND
		get_viewport().transparent_bg = true
		print("passthrough enabled (alpha blend)")
	else:
		print("alpha-blend passthrough not supported; available modes: ", modes)
