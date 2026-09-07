@tool
extends EditorPlugin

# Editor-side half of the addon: registers the Android export guard. The runtime half
# (ArucoMarkerTracking) is a plain scene node and needs no editor plumbing -- class_name
# already puts it in the node dialog.

const GuardScript := preload("res://addons/opencv_aruco/export_check.gd")

var _guard: EditorExportPlugin


func _enter_tree() -> void:
	_guard = GuardScript.new()
	add_export_plugin(_guard)


func _exit_tree() -> void:
	remove_export_plugin(_guard)
	_guard = null
