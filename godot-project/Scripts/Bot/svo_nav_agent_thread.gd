extends RigidBody3D

var thread: Thread
var request_queue = []
var result_queue = {}
var mutex = Mutex.new()
var query_id_counter = 0

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
	thread = Thread.new()
	# temp solution, replace with mamager when ready for games
	svo = $"../spider exp project/SvoNavmesh"

func _process(delta):
	_process_thread_queue()

func _process_thread_queue():
	mutex.lock()
	for query in request_queue:
		var result = can_travel_directly_with_ray(query)
		result_queue[query["id"]] = result
	request_queue.clear()
	mutex.unlock()

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
			#thread.start(_threaded_pathfinding.bind(data))

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

func find_path_v2(start, end, agent_r, is_smooth):
	if svo.query_voxel(start) || svo.query_voxel(end):
		print("Point inside SOLID!");
		return;
		
	# check direct path
	var query_id = query_id_counter
	query_id_counter += 1
	var query_data = {"from": start, "to": end, "id": query_id}
	var can_traverse = can_travel_directly_with_ray(query_data)
	
	if can_traverse:
		path.clear()
		path.append(start)
		path.append(end)
		print(("Direct path found."))
	else:
		#find_raw_path(start, end, agent_r);
		if path.is_empty():
			print("Path finding failed!")
			return;
		
		if is_smooth:
			#smooth_path_string_pulling_fast_v2(agent_r)
			pass
			
		#if svo.debug_mode: 
			#init_debug_path(agent_r * debug_path_scale)
		

func _threaded_function():
	for i in range(10):  # 示例，假设有10个查询请求
		var query_data = {"from": Vector3(0, 10, 0), "to": Vector3(0, -10, 0), "id": i}
		mutex.lock()
		request_queue.append(query_data)
		mutex.unlock()
		# 等待主线程处理结果
		while not result_queue.has(i):
			OS.delay_msec(10)
		var result = result_queue[i]
		print("Result for query ", i, ": ", result)
		mutex.lock()
		result_queue.erase(i)
		mutex.unlock()

func can_travel_directly_with_ray(query):
	var space_state = get_world_3d().direct_space_state
	var query_parameters = PhysicsRayQueryParameters3D.new()
	query_parameters.from = query["from"]
	query_parameters.to = query["to"]
	var result = space_state.intersect_ray(query_parameters)
	
	if (!result.is_empty()):
		var result_rid = result["rid"];
		if (svo.target_rids.has(result_rid)):
			return false;
	return true;
