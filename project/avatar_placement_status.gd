extends Node
## User-facing placement status for either the headset Label3D or desktop Label.

@export var avatar_rig: Node3D

func _process(_delta: float) -> void:
	var placed := avatar_rig != null and avatar_rig.is_visible_in_tree()
	set("text", "Avatar placed" if placed else "Placing avatar…")
