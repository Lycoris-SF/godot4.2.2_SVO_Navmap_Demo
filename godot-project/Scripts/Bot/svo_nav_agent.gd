extends RigidBody3D

var thread: Thread
var agent_r_temp = 0.125
var svo : SvoNavmesh
var path = []
var current_target_index = 0
var close_enough_radius = 0.125/2
var max_speed = 0.35
var arrival_threshold = 0.125*7  # 减速开始的距离阈值
var max_acceleration = 9.81 * 0.7  # 最大加速度, 3G
var max_deceleration = 9.81 * 0.5  # 最大减速度, 1G

func _ready():
	#thread = Thread.new()
	# temp solution, replace with mamager when ready for games
	svo = $"../spider exp project/SvoNavmesh"
			
func _exit_tree():
	#thread.wait_to_finish()
	pass

func _integrate_forces(state):
	if path.size() > 0 and current_target_index < path.size():
		var target = path[current_target_index]
		var to_target = target - self.position
		var distance = to_target.length()

		if distance < close_enough_radius:
			current_target_index += 1	# 移向下一个目标点
			if current_target_index >= path.size():
				print("Path completed!")
				return
		else:
			to_target = to_target.normalized()
			var target_speed = max_speed
			if distance < arrival_threshold:
				# 线性减速到0
				target_speed = lerp(max_speed, max_speed/5, 1.0 - distance / arrival_threshold)

			var desired_velocity = to_target * target_speed
			var velocity_change = desired_velocity - state.linear_velocity
			var impulse = velocity_change * mass / state.step	# 使用物理步长正规化冲量

			# 根据是加速还是减速应用不同的限制
			var current_speed = state.linear_velocity.length()
			var impulse_magnitude = impulse.length()
			if desired_velocity.length() > current_speed:
				# 加速情况
				var max_impulse = max_acceleration * mass * state.step
				if impulse_magnitude > max_impulse:
					impulse = impulse.normalized() * max_impulse
			else:
				# 减速情况
				var max_impulse = max_deceleration * mass * state.step
				if impulse_magnitude > max_impulse:
					impulse = impulse.normalized() * max_impulse

			# 应用冲量
			state.apply_central_impulse(impulse)

func _input(event):
	# Receives key input
	if event is InputEventKey and event.pressed:
		# it's too early for multi thread
		if event.keycode == KEY_F:
			var rand_empty_in_svo = _find_rand_empty_pos()
			current_target_index = 0
			path = svo.find_path(self.position, rand_empty_in_svo, agent_r_temp, true)
		if event.keycode == KEY_M:
			var rand_empty_in_svo = _find_rand_empty_pos()
			var rand_empty2_in_svo = _find_rand_empty_pos()
			var data = [rand_empty_in_svo, rand_empty2_in_svo, agent_r_temp]
			thread.start(_threaded_pathfinding.bind(data))

func _threaded_pathfinding(userdata):
	var is_smooth = true
	svo.find_path_multi_thread(userdata[0], userdata[1], userdata[2], is_smooth)

func _find_rand_empty_pos():
	var root_size = svo.rootVoxelSize
	var root_pos = svo.global_position
	var half_size = root_size * 0.5
	var random_point = Vector3()
	var max_attempts = 100
	var attempts = 0
	
	while attempts < max_attempts:
		var random_x = randf_range(-half_size, half_size)
		var random_y = randf_range(-half_size, half_size)
		var random_z = randf_range(-half_size, half_size)
		random_point = Vector3(random_x, random_y, random_z) + root_pos
		
		if not svo.query_voxel(random_point):
			return random_point
			
		attempts += 1
		
	print("Failed to find an empty position after", max_attempts, "attempts.")
	return null
