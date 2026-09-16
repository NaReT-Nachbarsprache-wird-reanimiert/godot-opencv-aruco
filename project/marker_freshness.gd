class_name MarkerFreshness
extends RefCounted
## The app's view of "which ArUco markers are usable right now", on top of the addon's id-keyed API.
##
## Two different questions, and both the avatar rig and the gizmo ask both of them:
##
##   result_ids() -- the ids that came out of the SINGLE newest detection. The common pose is only
##                   ever fused from one result: two results were baked with two different head
##                   poses, so mixing them mixes two capture-time corrections.
##   age_ms(id)   -- how stale one marker is on its own, regardless of which result it came from.
##
## BEFORE THE ADDON this was inference. main_3d.gd wrote a marker node's global_transform only when
## OpenCV had returned that id, so "the node's transform changed" stood in for "the marker was just
## detected" (marker_detection_stamp.gd), and same-result grouping was done by comparing the
## millisecond stamps for exact equality. ArucoMarkerTracking answers both directly: markers_updated
## carries exactly the ids of one detection, and marker_age_ms is a real per-marker age. So this
## class no longer infers anything -- what is left is the app's POLICY.
##
## That policy is the 300 ms window below, and it is deliberately SHORTER than the addon's
## marker_lost_timeout_ms (500 ms, after which an XR tracker is paused). A marker that is still good
## enough for a tracker to hold its last pose is not automatically good enough to feed a fresh
## fusion, and the rig and the gizmo must agree on where that line is -- hence one place for it
## rather than a constant in each.

## How long a detection result stays usable. Shorter than the addon's tracker grace period on
## purpose; see the class comment.
var tracking_loss_timeout_ms := 300

var _tracking: ArucoMarkerTracking = null
## The ids this consumer cares about. A detection that found only OTHER markers must not count as
## a fresh result here, the same way it would not have moved this consumer's marker nodes before.
var _watched_ids: Array = []
var _result_ids: Array = []
var _result_ms := -1


## Subscribe to the tracking node. Call from the consumer's _ready, once its exports are applied.
func attach(tracking: ArucoMarkerTracking, watched_ids: Array) -> void:
	_tracking = tracking
	_watched_ids = watched_ids.duplicate()
	if _tracking == null:
		push_error("[cpr] [freshness::attach] no ArucoMarkerTracking assigned; markers stay unseen")
		return
	if not _tracking.markers_updated.is_connected(_on_markers_updated):
		_tracking.markers_updated.connect(_on_markers_updated)


## Ids from the single newest detection, or [] once that result has aged out. Empty is the caller's
## cue that tracking is lost -- not that the markers moved.
func result_ids() -> Array:
	if _result_ms < 0 or Time.get_ticks_msec() - _result_ms > tracking_loss_timeout_ms:
		return []
	return _result_ids


## Timestamp of the newest result, in Time.get_ticks_msec(). One value per detection, which is what
## the provider and the stabilizer use to take exactly one sample per OpenCV result rather than one
## per rendered frame.
func result_ms() -> int:
	return _result_ms


## Milliseconds since this marker was last detected; INF if it never was. INF compares correctly
## against any window, so a never-seen id is simply never fresh.
func age_ms(id: int) -> float:
	if _tracking == null:
		return INF
	return _tracking.marker_age_ms(id)


## True while this marker's own pose is inside the freshness window, independent of result grouping.
func is_fresh(id: int) -> bool:
	return age_ms(id) <= float(tracking_loss_timeout_ms)


func _on_markers_updated(ids: Array) -> void:
	var mine: Array = []
	for id in ids:
		if _watched_ids.has(id):
			mine.append(id)
	# A detection that found none of our markers is not a result for us. Letting it through would
	# refresh _result_ms without refreshing any pose, so a marker that left view would look fresh
	# for as long as any OTHER marker stayed visible.
	if mine.is_empty():
		return
	_result_ids = mine
	_result_ms = Time.get_ticks_msec()
