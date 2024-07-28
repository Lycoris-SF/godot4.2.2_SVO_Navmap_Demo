extends RigidBody3D

signal A_star_completed
signal Path_found

# thread(only one in background)
var thread: Thread
var is_smooth
var is_physics_task_set = false

# Microbenchmarking 
var begin_time_u
var begin_time_m
var end_time_u
var end_time_m

# steering
var agent_r_temp = 0.125
var svo : SvoNavmesh
var path = []
var current_target_index = 0
var close_enough_radius = 0.125/2
var max_speed = 0.35
var arrival_threshold = 0.125*7  # Distance threshold for deceleration to start
var max_acceleration = 9.81 * 0.7  # Maximum acceleration, 3G
var max_deceleration = 9.81 * 0.5  # Maximum deceleration, 1G

func _ready():
	thread = Thread.new()
	# temp solution, replace with mamager when ready for games
	svo = $"../spider exp project/SvoNavmesh"
	A_star_completed.connect(_on_A_star_completed)
	Path_found.connect(_on_path_found)
		
func _physics_process(delta):
	if is_physics_task_set:
		PF_physics_task()
			
func _exit_tree():
	thread.wait_to_finish()

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
			find_path_multi_thread()

func record_time(string):
	if end_time_m - begin_time_m < 1:
		print(string, end_time_u - begin_time_u, " microseconds")
	else:
		print(string, end_time_m - begin_time_m, " milliseconds")

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

func find_path_multi_thread():
	is_smooth = true
	current_target_index = 0
	path.clear()
	var rand_empty_in_svo = _find_rand_empty_pos()
	var data = [self.position, rand_empty_in_svo, agent_r_temp]
	var direct_path = svo.direct_path_check(self.position, rand_empty_in_svo, agent_r_temp)
	if not direct_path:
		thread.start(A_star.bind(data))
	else:
		emit_signal("Path_found")
		
func A_star(data):
	begin_time_u = Time.get_ticks_usec()
	begin_time_m = Time.get_ticks_msec()

	if not svo.find_raw_path(data[0],data[1],data[2]):
		print("Path finding failed!")
		call_deferred("emit_signal", "A_star_completed")
		return false
	else:
		call_deferred("emit_signal", "A_star_completed")
		end_time_u = Time.get_ticks_usec()
		end_time_m = Time.get_ticks_msec()
		record_time("Path finding took: ")
		return true

func _on_A_star_completed():
	var result = thread.wait_to_finish()
	is_physics_task_set = result and is_smooth
	if result and not is_physics_task_set:
		svo.transfer_path_result()
		init_debug_path()

func PF_physics_task():
	begin_time_u = Time.get_ticks_usec()
	begin_time_m = Time.get_ticks_msec()
	
	svo.smooth_path_string_pulling_fast_v2(agent_r_temp)
	svo.transfer_path_result()
	
	end_time_u = Time.get_ticks_usec()
	end_time_m = Time.get_ticks_msec()
	record_time("Smooth path took: ")
	
	is_physics_task_set = false
	call_deferred("init_debug_path")
	
func init_debug_path():
	if svo.debug_mode:
		begin_time_u = Time.get_ticks_usec()
		begin_time_m = Time.get_ticks_msec()
		
		svo.init_debug_path(agent_r_temp*svo.debug_path_scale)
		
		end_time_u = Time.get_ticks_usec()
		end_time_m = Time.get_ticks_msec()
		record_time("Init rendering took: ")
		
	emit_signal("Path_found")
		
func _on_path_found():
	path = svo.get_last_path_result()
