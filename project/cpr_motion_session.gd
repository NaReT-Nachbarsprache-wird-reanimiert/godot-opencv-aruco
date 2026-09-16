extends RefCounted
## Sensor-free practice state. Excursion is hand travel, not measured chest depth.
signal beat_requested

var target_bpm := 110.0
var breathing_duration_s := 5.0
var minimum_travel_m := 0.050  # BLS standard: ~5cm hand excursion per compression
var return_tolerance_m := 0.010
var active := false
var phase := "compressions"
var count := 0
var total_count := 0
var breathing_remaining_s := 0.0
var travel_m := 0.0
var last_stroke_travel_m := 0.0
var tracking_available := false
var _baseline_ready := false
var _top := 0.0
var _bottom := 0.0
var _descending := false
var _stroke_time := 0.0
var _candidate_count := 0
var _since_stroke := 99.0
var _beat_remaining := 0.0
var _last_beat_time := -99.0  # When last beep occurred

func reset() -> void:
	active = false
	phase = "compressions"
	count = 0
	total_count = 0
	breathing_remaining_s = 0.0
	last_stroke_travel_m = 0.0
	_since_stroke = 99.0
	_beat_remaining = 0.0
	invalidate_tracking()

func start() -> void:
	reset()
	active = true

func invalidate_tracking() -> void:
	tracking_available = false
	_baseline_ready = false
	_descending = false
	_stroke_time = 0.0
	_candidate_count = 0
	travel_m = 0.0

func update(delta: float, valid: bool, height_m: float, paused := false) -> void:
	if paused:
		invalidate_tracking()
		return
	delta = maxf(delta, 0.0)
	_since_stroke += delta
	if phase == "breathing":
		invalidate_tracking()
		breathing_remaining_s = maxf(0.0, breathing_remaining_s - delta)
		if breathing_remaining_s <= 0.0:
			phase = "compressions"
			count = 0
			_beat_remaining = 0.0
		return
	# Beat timing is independent of hand movements and continues through hand occlusion.
	var now := Time.get_ticks_msec() / 1000.0
	_beat_remaining -= delta
	if _beat_remaining <= 0.0:
		beat_requested.emit()
		_last_beat_time = now
		var interval := 60.0 / clampf(target_bpm, 100.0, 120.0)
		_beat_remaining = interval + fmod(_beat_remaining, interval)
	# Never bridge an occlusion or a long render stall into a counted stroke.
	if not valid or not is_finite(height_m) or delta > 0.25:
		invalidate_tracking()
		return
	tracking_available = true
	if not _baseline_ready:
		_top = height_m
		_bottom = height_m
		_baseline_ready = true
		return
	if not _descending:
		_top = maxf(_top, height_m)
		if _top - height_m > 0.004:
			_descending = true
			_bottom = height_m
			_stroke_time = 0.0
	else:
		_stroke_time += delta
		_bottom = minf(_bottom, height_m)
	travel_m = maxf(0.0, _top - height_m)
	if not _descending:
		return
	if _stroke_time > 1.5 or _top - _bottom > 0.12:
		invalidate_tracking()
		return
	# Require a full return near the top, not merely a change of velocity at the bottom.
	if height_m >= _top - return_tolerance_m and height_m - _bottom >= 0.006:
		var excursion := _top - _bottom
		var time_since_beat := absf(now - _last_beat_time)
		var is_on_beat := (not active) or (time_since_beat <= 0.2)
		if excursion >= minimum_travel_m and is_on_beat:
			last_stroke_travel_m = excursion
			if not active:
				active = true
				count = 1
				total_count = 1
				_beat_remaining = 0.0
			else:
				count += 1
				total_count += 1
			_since_stroke = 0.0
			if count >= 30:
				phase = "breathing"
				breathing_remaining_s = breathing_duration_s
				invalidate_tracking()
		_top = height_m
		_bottom = height_m
		_descending = false
		travel_m = 0.0
