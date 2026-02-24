#ifndef AnimView_h
#define AnimView_h

#include "ModelView.h"

struct AnimView : ModelView {
	float lastT    = 0;
	bool  animating = false;

	std::function<void()>      initFunction    = []() {};
	std::function<void(float)> frameFunction   = [](float) {};
	std::function<void(int)>   keyFunction     = [](int) {};
	std::function<void(char)>  controlFunction = [](char) {};

	AnimView(float x, float y, float w, float h, const std::string& name = "")
		: ModelView(x, y, w, h, name) {}

	// Handle events forwarded from GLFW callbacks
	virtual bool handle(int e) override;

	// Advance animation frame (called directly from the GLFW render loop)
	void update(float dt);
};

#endif /* AnimView_h */
