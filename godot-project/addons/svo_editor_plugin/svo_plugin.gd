@tool
extends EditorPlugin

var dock_content

func _enter_tree():
	var dock_scene = preload("res://addons/svo_editor_plugin/svo_plugin.tscn")
	dock_content = dock_scene.instantiate()
	add_control_to_dock(DOCK_SLOT_RIGHT_UL, dock_content)
	#add_control_to_container(EditorPlugin.CONTAINER_CANVAS_EDITOR_SIDE_RIGHT, dock_scene)
	#dock_content.pressed.connect(_on_rebuild_pressed)
	dock_content.get_node("HBoxContainer Rebuild/Rebuild Button").pressed.connect(_on_rebuild_pressed)
	dock_content.get_node("HBoxContainer Rebuild/Refresh Button").pressed.connect(_on_refresh_pressed)
	dock_content.get_node("HBoxContainer Clear/Clear Button").pressed.connect(_on_clear_svo_pressed)
	dock_content.get_node("HBoxContainer Clear/ClearAll Button").pressed.connect(_on_clear_all_pressed)
	dock_content.get_node("HBoxContainer Insert/Insert Button").pressed.connect(_on_insert_pressed)
	dock_content.get_node("HBoxContainer Update/Update Button").pressed.connect(_on_update_pressed)
	dock_content.get_node("HBoxContainer Path End/Find Path Button").pressed.connect(_on_find_path_pressed)

func _exit_tree():
	remove_control_from_docks(dock_content)
	dock_content.queue_free()

func _on_rebuild_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node and selected_node is SvoNavmesh:
		selected_node.rebuild_svo()
func _on_refresh_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node and selected_node is SvoNavmesh:
		selected_node.refresh_svo()
func _on_clear_all_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node and selected_node is SvoNavmesh:
		selected_node.clear_svo(true)
func _on_clear_svo_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node and selected_node is SvoNavmesh:
		selected_node.clear_svo(false)
		
func _on_insert_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node and selected_node is SvoNavmesh:
		var spin_box_x = dock_content.get_node("HBoxContainer Insert/SpinBoxX").get_value()
		var spin_box_y = dock_content.get_node("HBoxContainer Insert/SpinBoxY").get_value()
		var spin_box_z = dock_content.get_node("HBoxContainer Insert/SpinBoxZ").get_value()

		var position = Vector3(spin_box_x, spin_box_y, spin_box_z)
		selected_node.insert_voxel(position)
func _on_update_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node and selected_node is SvoNavmesh:
		var spin_box_x = dock_content.get_node("HBoxContainer Update/SpinBoxX").get_value()
		var spin_box_y = dock_content.get_node("HBoxContainer Update/SpinBoxY").get_value()
		var spin_box_z = dock_content.get_node("HBoxContainer Update/SpinBoxZ").get_value()
		var is_solid = dock_content.get_node("HBoxContainer Update/CheckBox").is_pressed()

		var position = Vector3(spin_box_x, spin_box_y, spin_box_z)
		selected_node.update_voxel(position,is_solid)
		
func _on_find_path_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node and selected_node is SvoNavmesh:
		var start_x = dock_content.get_node("HBoxContainer Path Start/SpinBoxX").get_value()
		var start_y = dock_content.get_node("HBoxContainer Path Start/SpinBoxY").get_value()
		var start_z = dock_content.get_node("HBoxContainer Path Start/SpinBoxZ").get_value()
		var end_x = dock_content.get_node("HBoxContainer Path End/SpinBoxX").get_value()
		var end_y = dock_content.get_node("HBoxContainer Path End/SpinBoxY").get_value()
		var end_z = dock_content.get_node("HBoxContainer Path End/SpinBoxZ").get_value()
		var radius = dock_content.get_node("HBoxContainer Path Start/SpinBoxR").get_value()

		var start = Vector3(start_x, start_y, start_z)
		var end = Vector3(end_x, end_y, end_z)
		selected_node.find_path_and_draw(start,end,radius)
