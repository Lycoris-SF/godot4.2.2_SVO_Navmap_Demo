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
	dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Button/Insert Button").pressed.connect(_on_insert_pressed)
	dock_content.get_node("HBoxContainer Update/Update Button").pressed.connect(_on_update_pressed)
	dock_content.get_node("HBoxContainer Path End/Find Path Button").pressed.connect(_on_find_path_pressed)
	dock_content.get_node("HBoxContainer ID Query/Check Voxel Info Button").pressed.connect(_on_check_id_pressed)
	dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Button").connect("gui_input", _on_gui_input)
	dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Button/PopupMenu").connect("id_pressed", _on_popup_id_pressed)
	#dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Button").get_popup().id_pressed.connect(_on_popup_id_pressed)

func _exit_tree():
	remove_control_from_docks(dock_content)
	dock_content.queue_free()

func _on_popup_id_pressed(id):
	print("Godot clipboard is a mess, don't use it")
	return
	if id == 0: # Copy
		var x_value = dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Value/SpinBoxX").get_value()
		var y_value = dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Value/SpinBoxY").get_value()
		var z_value = dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Value/SpinBoxZ").get_value()
		var copy_value = {"x": x_value, "y": y_value, "z": z_value}
		DisplayServer.clipboard_set(copy_value)
	elif id == 1: # Paste
		print("show sth")
		var paste_value = DisplayServer.clipboard_get()
		print(paste_value)
		if typeof(paste_value) == TYPE_DICTIONARY:
			dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Value/SpinBoxX").set_value(paste_value.find("x"))
			dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Value/SpinBoxY").set_value(paste_value.find("y"))
			dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Value/SpinBoxZ").set_value(paste_value.find("z"))

func _on_gui_input(event):
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_RIGHT and event.pressed:
		var mouse_pos = get_viewport().get_mouse_position()
		var rect_width = 100
		var rect_height = 100
		var rect = Rect2i(mouse_pos.x, mouse_pos.y, rect_width, rect_height)
		dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Button/PopupMenu").popup(rect)

func _on_rebuild_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node is SvoNavmesh:
		selected_node.rebuild_svo()
func _on_refresh_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node is SvoNavmesh:
		selected_node.refresh_svo()
func _on_clear_all_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node is SvoNavmesh:
		selected_node.clear_svo(true)
func _on_clear_svo_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node is SvoNavmesh:
		selected_node.clear_svo(false)
		
func _on_insert_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node is SvoNavmesh:
		var spin_box_x = dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Value/SpinBoxX").get_value()
		var spin_box_y = dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Value/SpinBoxY").get_value()
		var spin_box_z = dock_content.get_node("VBoxContainer_Insert/HBoxContainer_Value/SpinBoxZ").get_value()

		var position = Vector3(spin_box_x, spin_box_y, spin_box_z)
		selected_node.insert_voxel(position)
func _on_update_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node is SvoNavmesh:
		var spin_box_x = dock_content.get_node("HBoxContainer Update/SpinBoxX").get_value()
		var spin_box_y = dock_content.get_node("HBoxContainer Update/SpinBoxY").get_value()
		var spin_box_z = dock_content.get_node("HBoxContainer Update/SpinBoxZ").get_value()
		var is_solid = dock_content.get_node("HBoxContainer Update/CheckBox").is_pressed()

		var position = Vector3(spin_box_x, spin_box_y, spin_box_z)
		selected_node.update_voxel(position,is_solid)
		
func _on_check_id_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node is SvoNavmesh:
		var line_edit = dock_content.get_node("HBoxContainer ID Query/LineEdit ID") as LineEdit
		var id = line_edit.text
		
		selected_node.check_voxel_with_id(id)
		
func _on_find_path_pressed():
	var editor_selection = get_editor_interface().get_selection()
	var selected_node = editor_selection.get_selected_nodes()[0]
	if selected_node is SvoNavmesh:
		var start_x = dock_content.get_node("HBoxContainer Path Start/SpinBoxX").get_value()
		var start_y = dock_content.get_node("HBoxContainer Path Start/SpinBoxY").get_value()
		var start_z = dock_content.get_node("HBoxContainer Path Start/SpinBoxZ").get_value()
		var end_x = dock_content.get_node("HBoxContainer Path End/SpinBoxX").get_value()
		var end_y = dock_content.get_node("HBoxContainer Path End/SpinBoxY").get_value()
		var end_z = dock_content.get_node("HBoxContainer Path End/SpinBoxZ").get_value()
		var radius = dock_content.get_node("HBoxContainer Path Start/SpinBoxR").get_value()
		var is_smooth = dock_content.get_node("HBoxContainer Path End/CheckBox").is_pressed()

		var start = Vector3(start_x, start_y, start_z)
		var end = Vector3(end_x, end_y, end_z)
		selected_node.find_path(start,end,radius,is_smooth)
