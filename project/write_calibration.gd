extends SceneTree
## Turns the offsets learned by make_calibration.py into a navel_calibration.cfg.
##
## Run headless:
##   godot --headless --script write_calibration.gd -- OFFSETS.json OLD.cfg NEW.cfg
##
## Godot writes a Transform3D as twelve bare numbers, and getting its basis convention wrong would
## transpose every rotation without any error being raised. So the file is built here, with Godot's
## own types doing the spelling, rather than by formatting text in Python and hoping.
##
## The existing file is loaded first and only the [common] offsets are replaced. That deliberately
## preserves [body] rest_basis: the offsets are rigid facts about markers glued to the mannequin,
## while the rest orientation describes where the mannequin is currently standing, and only the
## first kind can be learned from a recording made earlier.
##
## KEYS ARE MARKER IDS. Offsets used to be keyed by the scene node that carried the marker pose
## ("aruco_patch0"); the opencv_aruco addon addresses markers by id, so the cfg key is "marker_0"
## (CommonPoseProvider.offset_key_for -- an identifier rather than a bare number, for the reason
## given there). A JSON file still using the old spelling is accepted and converted, because the
## recordings those offsets were solved from are expensive and predate the rename.


func _init() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() != 3:
		push_error("Need OFFSETS.json OLD.cfg NEW.cfg")
		quit(1)
		return

	var text := FileAccess.get_file_as_string(args[0])
	if text.is_empty():
		push_error("Could not read %s" % args[0])
		quit(1)
		return
	var offsets: Dictionary = JSON.parse_string(text)

	var cfg := ConfigFile.new()
	# Not an error if it is missing - then this simply writes a fresh calibration with no rest.
	cfg.load(args[1])

	for json_key in offsets:
		var key := _marker_key(json_key)
		if key.is_empty():
			push_error("Offset key %s is neither a marker id nor aruco_patch<id>" % json_key)
			quit(1)
			return
		var quaternion: Array = offsets[json_key]["quaternion"]
		var origin: Array = offsets[json_key]["origin"]
		cfg.set_value("common", key, Transform3D(
			Basis(Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])),
			Vector3(origin[0], origin[1], origin[2])
		))
		print("marker %s  origin %.1f mm" % [
			key,
			Vector3(origin[0], origin[1], origin[2]).length() * 1000.0,
		])

	if cfg.save(args[2]) != OK:
		push_error("Could not write %s" % args[2])
		quit(1)
		return
	print("wrote ", args[2], "   rest_basis kept: ", cfg.has_section_key("body", "rest_basis"))
	quit()


## The cfg key for one JSON entry. Accepts a bare id ("0") and the pre-addon node name
## ("aruco_patch0"), and spells both the way the loader looks them up; returns "" for anything
## else rather than writing a key nothing will read.
func _marker_key(json_key: String) -> String:
	if json_key.is_valid_int():
		return CommonPoseProvider.offset_key_for(json_key.to_int())
	if json_key.begins_with("aruco_patch"):
		var suffix := json_key.substr("aruco_patch".length())
		if suffix.is_valid_int():
			return CommonPoseProvider.offset_key_for(suffix.to_int())
	return ""
