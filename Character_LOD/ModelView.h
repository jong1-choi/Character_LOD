#ifndef ModelView_h
#define ModelView_h

#include "Widget.h"
#include "GLTools.h"
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <functional>

struct FB {
	size_t w = 0;
	size_t h = 0;
	GLuint fbo   = 0;
	GLuint color = 0;
	GLuint depth = 0;

	GLint oldVP[4];
	GLint oldFB, oldSc;

	static void setTexParam(GLuint minFilter = GL_LINEAR, GLuint warp = GL_CLAMP_TO_EDGE);

	void clearGL();
	void create(int ww, int hh);
	void setToTarget();
	void restoreVP();
	void bindColor(GLuint prog, const std::string& name, GLuint slot);
	void bindDepth(GLuint prog, const std::string& name, GLuint slot);
};


struct View3D : Widget {
	float dist  = 100;
	float yaw   = 0;
	float pitch = 0.3f;
	float fov   = 1.f;
	glm::vec2 oldPt;
	glm::vec3 oldPt3;
	glm::vec3 sceneCenter = { 0, 0, 0 };
	glm::vec3 iblCoeffs[9];

	std::function<bool(const std::string fileName)> getFileName = [](const std::string fileName) { return false; };
	std::function<bool(const glm::vec2& pt2, const glm::vec3&, float)> pressFunc   = [](const glm::vec2&, const glm::vec3&, float) { return false; };
	std::function<bool(const glm::vec2& pt2, const glm::vec3&, float)> dragFunc    = [](const glm::vec2&, const glm::vec3&, float) { return false; };
	std::function<bool(const glm::vec2& pt2, const glm::vec3&, float)> releaseFunc = [](const glm::vec2&, const glm::vec3&, float) { return false; };

	View3D(float x, float y, float w, float h, const std::string& n);

	virtual bool handle(int e) override;

	virtual glm::mat4 getViewMat() const;
	virtual glm::mat4 getProjMat() const;
	virtual void setViewProj(GLuint prog, const std::string& viewName = "viewMat", const std::string& projName = "projMat");

	glm::vec2 toScreen(const glm::vec3& p, const glm::mat4& modelMat = glm::mat4(1)) const;
	std::pair<glm::vec3, float> unproject(const glm::vec2& ptS);
	glm::vec3 unproject(const glm::vec2& ptS, float d);
	glm::vec3 projectN(const glm::vec3& pt3);
};

struct ModelView : View3D {
	ModelView(float x, float y, float w, float h, const std::string& name = "")
		: View3D(x, y, w, h, name) {}

	bool      enableShadow = true;
	glm::vec3 lightPos     = { 0, 200, 250 };

	GLuint renderProg = 0, renderVert = 0, renderFrag = 0;
	GLuint const_Prog = 0, const_Vert = 0, const_Frag = 0;

	std::function<void()> renderFunction = []() {};
	std::function<void()> wireFunction   = []() {};

	FB shadowMap;

	virtual void drawGL() override;
};


#endif /* ModelView_h */
