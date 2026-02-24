#include "AnimView.h"


bool AnimView::handle(int e) {
	if (e == EVENT_KEYDOWN) {
		keyFunction(g_input.key);
		if (g_input.key == ' ') {
			animating = !animating;
			if (animating)
				lastT = (float)glfwGetTime();
			return true;
		}
		else if (g_input.key == '0') {
			animating = false;
			initFunction();
			return true;
		}
		else if (g_input.key == '[' || g_input.key == ']') {
			controlFunction((char)g_input.key);
			return true;
		}
	}
	return ModelView::handle(e);
}

void AnimView::update(float dt) {
	frameFunction(dt);
}
