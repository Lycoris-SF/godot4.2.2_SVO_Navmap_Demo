extends Control

func _on_StartButton1_pressed():
	GlobalValue.svo_debugMode = $CenterContainer/HBoxContainer/CheckButton.button_pressed
	get_tree().change_scene_to_file("res://Scene/SVO_demo.tscn")
	
func _on_StartButton2_pressed():
	get_tree().change_scene_to_file("res://Scene/SVO_demo_simplified.tscn")

func _on_debugMode_changed():
	pass
	
