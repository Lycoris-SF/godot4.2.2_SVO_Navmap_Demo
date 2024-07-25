extends SvoNavmesh

var thread: Thread
var agent_r_temp = 0.05
	
func _ready():
	thread = Thread.new()
	if self is SvoNavmesh and self.is_visible_in_tree():
		self.clear_svo(false)
		self.refresh_svo()
		if not Engine.is_editor_hint():
			self.rebuild_svo()
			
func _exit_tree():
	thread.wait_to_finish()
	
func _input(event):
	# Receives key input
	if event is InputEventKey and event.pressed:
		# it's too early for multi thread
		if event.keycode == KEY_F:
			if self is SvoNavmesh and self.is_visible_in_tree():
				var rand_empty_in_svo = _find_rand_empty_pos()
				var rand_empty2_in_svo = _find_rand_empty_pos()
				self._find_path(rand_empty_in_svo, rand_empty2_in_svo, agent_r_temp)
		if event.keycode == KEY_M:
			if self is SvoNavmesh and self.is_visible_in_tree():
				var rand_empty_in_svo = _find_rand_empty_pos()
				var rand_empty2_in_svo = _find_rand_empty_pos()
				var data = [rand_empty_in_svo, rand_empty2_in_svo, agent_r_temp]
				thread.start(_threaded_pathfinding.bind(data))
		if event.keycode == KEY_E:
			if self is SvoNavmesh and self.is_visible_in_tree():
				self._pathfinding_done()
			
func _start_threaded_rebuild():
	# it's too early for multi thread
	thread.start(_rebuild_svo)
				
func _rebuild_svo():
	self.rebuild_svo()

func _threaded_pathfinding(userdata):
	var is_smooth = true
	self.find_path_multi_thread(userdata[0], userdata[1], userdata[2], is_smooth)
	
func _find_path(start, end, agent_r):
	var is_smooth = true
	self.find_path(start, end, agent_r, is_smooth)
	call_deferred("_pathfinding_done")

func _find_rand_empty_pos():
	var root_size = self.rootVoxelSize
	var root_pos = self.global_position
	var half_size = root_size * 0.5
	var random_point = Vector3()
	var max_attempts = 100
	var attempts = 0
	
	while attempts < max_attempts:
		var random_x = randf_range(-half_size, half_size)
		var random_y = randf_range(-half_size, half_size)
		var random_z = randf_range(-half_size, half_size)
		random_point = Vector3(random_x, random_y, random_z) + root_pos
		
		if not self.query_voxel(random_point):
			return random_point
			
		attempts += 1
		
	print("Failed to find an empty position after", max_attempts, "attempts.")
	return null

func _pathfinding_done():
	var path = self.get_last_path_result()
	if path.is_empty():
		print("Path finding failed!")
	else:
		print("Path found: ")
		_print_formatted_path(path)
		
func _print_formatted_path(path):
	var formatted_path = []
	for point in path:
		formatted_path.append("(%f, %f, %f)" % [point.x, point.y, point.z])
	print("[%s]" % ", ".join(formatted_path))
