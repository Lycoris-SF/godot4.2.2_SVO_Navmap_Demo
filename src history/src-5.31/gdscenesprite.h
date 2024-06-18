#pragma once

#include <godot_cpp/classes/sprite2d.hpp>

namespace godot {

class GDSceneSprite : public Sprite2D {
	GDCLASS(GDSceneSprite, Sprite2D)

private:
	double time_passed;

protected:
	static void _bind_methods();

public:
	GDSceneSprite();
	~GDSceneSprite();

	void _process(double delta);
};

}
