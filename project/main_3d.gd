# Scene root for the CPR trainer.
#
# This file used to BE the pipeline: camera feed selection, the GPU->CPU readback, a hand-rolled
# detection Thread with its mutex and semaphore, the pose history and its CAMERA_LATENCY_MS guess,
# the OpenCV call with its nine arguments, the baking of camera-space poses into the aruco_patch
# nodes, and a TCP frame streamer -- ~340 lines that every consumer of the tracking had to be wired
# into. All of it is the opencv_aruco addon now (ArucoMarkerTracking, a sibling node in this
# scene), including the parts the old code could not do at all: the capture-time head pose from
# xrLocateSpace, the CameraX push path with real sensor timestamps, and publishing each marker as
# an OpenXRMarkerTracker on the XRServer.
#
# What is left here is the debug camera preview. The app's own consumers -- AvatarRig,
# MarkerGizmos, ArucoCsvLogger -- talk to the addon node directly through its exported reference,
# so nothing routes through this script any more.
#
# NOTE this scene deliberately does NOT spawn an XRAnchor3D per tracker the way the addon's demo
# consumer does (see addons/opencv_aruco/examples/marker_tracking_example.gd for that route). The
# CPR app draws markers with MarkerGizmos instead, and its avatar is placed from a FUSED pose of
# three markers rather than from any single tracker.
extends Node3D

@onready var cam_preview: TextureRect = $CameraLayer/CameraPreview
@onready var marker_tracking: ArucoMarkerTracking = $ArucoMarkerTracking


func _ready() -> void:
	# The texture is created once the camera feed is up; the getter covers the case where that
	# already happened before this _ready ran.
	marker_tracking.camera_feed_started.connect(_on_camera_feed_started)
	if marker_tracking.get_camera_texture() != null:
		_on_camera_feed_started(marker_tracking.get_camera_texture())


# Texture2D, not CameraTexture: the addon has two camera backends and they hand over different
# things -- the CameraServer path a live CameraTexture, the CameraX path an ImageTexture it updates
# per frame (and only when its camera_preview_enabled is on).
func _on_camera_feed_started(texture: Texture2D) -> void:
	cam_preview.texture = texture
