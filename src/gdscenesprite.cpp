#include "gdscenesprite.h"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void GDSceneSprite::_bind_methods() {
}

GDSceneSprite::GDSceneSprite() {
	// Initialize any variables here.
	time_passed = 0.0;
}

GDSceneSprite::~GDSceneSprite() {
	// Add your cleanup here.
}

void GDSceneSprite::_process(double delta) {
	time_passed += delta;

	Vector2 new_position = Vector2(10.0 + (10.0 * sin(time_passed * 2.0)), 40.0 + (40.0 * cos(time_passed * 1.5)));
	new_position = get_position() + Vector2(1.0f, 0);

	set_position(new_position);
}