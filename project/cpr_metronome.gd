extends AudioStreamPlayer
## Short PCM tone generated once; playback uses the normal Godot audio output.

func _ready() -> void:
	var sample_rate := 22050
	var duration := 0.055
	var frames := int(sample_rate * duration)
	var pcm := PackedByteArray()
	pcm.resize(frames * 2)
	for i in frames:
		var t := float(i) / sample_rate
		var envelope := minf(t / 0.005, (duration - t) / 0.010)
		var value := int(sin(TAU * 880.0 * t) * clampf(envelope, 0.0, 1.0) * 16000)
		pcm[i * 2] = value & 255
		pcm[i * 2 + 1] = (value >> 8) & 255
	var tone := AudioStreamWAV.new()
	tone.format = AudioStreamWAV.FORMAT_16_BITS
	tone.mix_rate = sample_rate
	tone.data = pcm
	stream = tone
	volume_db = -14.0
