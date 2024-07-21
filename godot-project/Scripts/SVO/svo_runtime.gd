extends SvoNavmesh

func _ready():
	if self is SvoNavmesh:
		self.clear_svo(false)
		self.refresh_svo()

func _input(event):
	# Receives key input
	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_B:
			if self is SvoNavmesh and self.is_visible_in_tree():
				self.rebuild_svo()
