# HEADLESS MEASUREMENT:
# 1. get_aabb() gets the small bounding box around one mesh.
# 2. Apply the mesh's global_transform so the box uses the mesh's
#    real position, rotation, and scale.
# 3. Repeat this for every mesh in the mannequin.
# 4. merge() combines all the small boxes into one large box.
# 5. box.size gives the mannequin's final X, Y, and Z dimensions.
extends SceneTree
func _initialize():
	var m = load("res://assets/avatar/mannequin.glb").instantiate()
	get_root().add_child(m)

	var mesh = m.find_children("*", "MeshInstance3D")[0]
	print("Size (m): ", (mesh.global_transform * mesh.get_aabb()).size)
	quit()
