extends SvoNavmesh

func _ready():
	if self is SvoNavmesh and self.is_visible_in_tree():
		self.clear_svo(false)
		self.refresh_svo()
		if not Engine.is_editor_hint():
			self.rebuild_svo()

func _input(event):
	# Receives key input
	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_B:
			if self is SvoNavmesh and self.is_visible_in_tree():
				self.rebuild_svo()
