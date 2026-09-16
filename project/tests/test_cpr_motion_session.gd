extends SceneTree
const Session = preload("res://cpr_motion_session.gd")
var failures := 0
var beats := 0

func _init() -> void:
	call_deferred("run")

func check(ok: bool, message: String) -> void:
	if not ok:
		failures += 1
		push_error(message)

func stroke(session, hz := 72, amplitude := 0.04) -> void:
	var frames := int(round(hz * 60.0 / 110.0))
	for i in range(frames + 1):
		var height := -amplitude * (1.0 - cos(TAU * float(i) / frames)) * 0.5
		session.update(1.0 / hz, true, height)

func run() -> void:
	for hz in [30, 72, 90]:
		var session = Session.new()
		for i in hz:
			session.update(1.0 / hz, true, 0.0)
		check(not session.active and session.count == 0, "Rest alone must not start CPR")
		stroke(session, hz)
		check(not session.active, "One stroke does not hide guide")
		stroke(session, hz)
		check(session.active and session.count == 2, "Two full strokes auto-start at %d Hz" % hz)
		for i in 28:
			stroke(session, hz)
		check(session.phase == "breathing" and session.count == 30, "30 strokes enter breathing phase")
		check(session.last_stroke_travel_m > 0.035, "Peak-to-top hand travel retained")
		var remaining: float = session.breathing_remaining_s
		session.update(1.0, false, 0.0, true)
		check(is_equal_approx(remaining, session.breathing_remaining_s), "App pause freezes breathing timer")
		session.update(2.0, false, 0.0)
		check(session.phase == "breathing" and session.breathing_remaining_s < remaining, "Breathing timer independent of hand availability")
		session.update(4.0, false, 0.0)
		check(session.phase == "compressions" and session.count == 0 and session.total_count == 30, "Next cycle resets count but preserves total")
		stroke(session, hz)
		check(session.count == 1, "Counting resumes after breathing")
	var jitter = Session.new()
	for i in 10:
		stroke(jitter, 72, 0.003)
	check(not jitter.active, "Small jitter must not count")
	jitter.start()
	jitter.update(0.016, true, 0.0)
	jitter.update(0.016, true, -0.03)
	jitter.update(0.016, false, 0.0)
	jitter.update(0.016, true, 0.0)
	check(jitter.count == 0, "Tracking gap cannot complete a stroke")
	stroke(jitter)
	check(jitter.count == 1, "Fresh complete stroke counts after tracking gap")
	var pacing = Session.new()
	pacing.start()
	pacing.beat_requested.connect(func(): beats += 1)
	for i in 720:
		pacing.update(1.0 / 72, false, 0.0)
	check(beats == 19, "110 BPM produces 19 paced beats in 10 seconds including initial beat")
	check(pacing.count == 0, "Metronome never creates compressions")
	var before := beats
	pacing.update(5.0, false, 0.0, true)
	check(beats == before, "No audio beats during application pause")
	pacing.phase = "breathing"
	pacing.breathing_remaining_s = 5.0
	pacing.update(1.0, false, 0.0)
	check(beats == before, "Metronome silent during breathing")
	print("CPR motion session tests: ", "PASS" if failures == 0 else "FAIL")
	quit(0 if failures == 0 else 1)
