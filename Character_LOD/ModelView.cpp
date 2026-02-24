#include "ModelView.h"


// --- FB ---

void FB::setTexParam(GLuint minFilter, GLuint warp) {
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, warp);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, warp);
}

void FB::clearGL() {
	if (color) glDeleteTextures(1, &color);    color = 0;
	if (depth) glDeleteTextures(1, &depth);    depth = 0;
	if (fbo)   glDeleteFramebuffers(1, &fbo);  fbo   = 0;
}

void FB::create(int ww, int hh) {
	if (ww == w && hh == h) return;
	w = ww;
	h = hh;
	printf("FB creation: %zu x %zu\n", w, h);
	clearGL();
	glGenTextures(1, &color);
	glBindTexture(GL_TEXTURE_2D, color);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ww, hh, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
	setTexParam();

	glGenTextures(1, &depth);
	glBindTexture(GL_TEXTURE_2D, depth);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, ww, hh, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
	setTexParam();

	glGenFramebuffers(1, &fbo);
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, &oldFB);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color, 0);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth, 0);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cerr << "FBO is incomplete";

	glBindFramebuffer(GL_FRAMEBUFFER, oldFB);
}

void FB::setToTarget() {
	glGetIntegerv(GL_VIEWPORT, oldVP);
	oldSc = glIsEnabled(GL_SCISSOR_TEST);
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &oldFB);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glDisable(GL_SCISSOR_TEST);
}

void FB::restoreVP() {
	glViewport(oldVP[0], oldVP[1], oldVP[2], oldVP[3]);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, oldFB);
	if (oldSc)
		glEnable(GL_SCISSOR_TEST);
}

void FB::bindColor(GLuint prog, const std::string& name, GLuint slot) {
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_2D, color);
	setUniform(prog, name, (int)slot);
}

void FB::bindDepth(GLuint prog, const std::string& name, GLuint slot) {
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_2D, depth);
	setUniform(prog, name, (int)slot);
}


// --- View3D ---

View3D::View3D(float x, float y, float w, float h, const std::string& n)
	: Widget(x, y, w, h, n) {
	iblCoeffs[0] = glm::vec3( 2.136610,  2.136610,  2.136610);
	iblCoeffs[1] = glm::vec3( 1.200665,  1.200665,  1.200665);
	iblCoeffs[2] = glm::vec3(-0.319614, -0.319614, -0.319614);
	iblCoeffs[3] = glm::vec3(-0.718034, -0.718034, -0.718034);
	iblCoeffs[4] = glm::vec3(-0.101370, -0.101370, -0.101370);
	iblCoeffs[5] = glm::vec3(-0.183365, -0.183365, -0.183365);
	iblCoeffs[6] = glm::vec3(-0.191558, -0.191558, -0.191558);
	iblCoeffs[7] = glm::vec3( 0.387255,  0.387255,  0.387255);
	iblCoeffs[8] = glm::vec3(-0.021659, -0.021659, -0.021650);
}

bool View3D::handle(int e) {
	if (e == EVENT_PUSH) {
		oldPt = g_input.mousePos;
		auto [pt3, d] = unproject(oldPt);
		oldPt3 = pt3;
		pressFunc(oldPt, oldPt3, d);
		return true;
	}
	else if (e == EVENT_DRAG) {
		glm::vec2 pt = g_input.mousePos;
		auto [pt3, d] = unproject(oldPt);
		oldPt3 = pt3;
		if (dragFunc(pt, pt3, d)) {
			return true;
		}
		yaw   += (pt.x - oldPt.x) / w() * 3.141592f;
		pitch += (pt.y - oldPt.y) / h() * 3.141592f;
		oldPt = pt;
		return true;
	}
	else if (e == EVENT_DND) {
		if (!g_input.droppedFiles.empty())
			getFileName(g_input.droppedFiles[0]);
		return true;
	}
	else if (e == EVENT_RELEASE) {
		glm::vec2 pt = g_input.mousePos;
		auto [pt3, d] = unproject(oldPt);
		oldPt3 = pt3;
		releaseFunc(pt, pt3, d);
	}
	else if (e == EVENT_SCROLL) {
		dist *= pow(0.8f, g_input.scrollDelta);
		return true;
	}
	return false;
}

glm::mat4 View3D::getViewMat() const {
	return glm::translate(glm::vec3(0, 0, -dist))
		* glm::rotate(pitch, glm::vec3(1, 0, 0))
		* glm::rotate(yaw, glm::vec3(0, 1, 0))
		* glm::translate(-sceneCenter);
}

glm::mat4 View3D::getProjMat() const {
	return glm::perspective(fov, w() / h(), 10.f, 2000.f);
}

void View3D::setViewProj(GLuint prog, const std::string& viewName, const std::string& projName) {
	setUniform(prog, "projMat", getProjMat());
	setUniform(prog, "viewMat", getViewMat());
}

glm::vec2 View3D::toScreen(const glm::vec3& p, const glm::mat4& modelMat) const {
	glm::vec4 p4 = getProjMat() * getViewMat() * modelMat * glm::vec4(p, 1);
	return ((glm::vec2(p4) / p4.w) * .5f + glm::vec2(.5)) * glm::vec2(w(), -h()) + glm::vec2(0, h());
}

std::pair<glm::vec3, float> View3D::unproject(const glm::vec2& ptS) {
	float d = 0;
	const float ratio = pxRatio();
	const glm::vec2 ptw = ptS + abs_pos();
	glReadPixels(
		(GLint)(ptw.x * ratio),
		(GLint)((g_input.winHeight - ptw.y) * ratio),
		1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &d);
	glm::vec3 pt3 = glm::vec3(ptS / glm::vec2(w(), h()), d) * 2.f - glm::vec3(1);
	pt3.y = -pt3.y;
	glm::vec4 ptW = inverse(getProjMat() * getViewMat()) * glm::vec4(pt3, 1);
	return { glm::vec3(ptW) / ptW.w, d };
}

glm::vec3 View3D::unproject(const glm::vec2& ptS, float d) {
	glm::vec3 pt3 = glm::vec3(ptS / glm::vec2(w(), h()), d) * 2.f - glm::vec3(1);
	pt3.y = -pt3.y;
	glm::vec4 ptW = inverse(getProjMat() * getViewMat()) * glm::vec4(pt3, 1);
	return glm::vec3(ptW) / ptW.w;
}

glm::vec3 View3D::projectN(const glm::vec3& pt3) {
	glm::vec4 ptS = getProjMat() * getViewMat() * glm::vec4(pt3, 1);
	return glm::vec3(ptS) / ptS.w;
}


// --- ModelView ---

void ModelView::drawGL() {
	glClearColor(1, 1, 1, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (renderProg == 0) {
		std::tie(renderProg, renderVert, renderFrag) = loadProgram("shader.vert", "shader.frag");
		std::tie(const_Prog, const_Vert, const_Frag) = loadProgram("const.vert", "const.frag");
	}

	glm::mat4 shadowV, shadowP;
	constexpr float shadowZNear = 100.f;
	constexpr float shadowZFar  = 10000.f;
	constexpr float shadowFov   = 1.0f;
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	if (enableShadow) {
		shadowMap.create(1024, 1024);
		shadowMap.setToTarget();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		shadowV = glm::lookAt(lightPos, sceneCenter, glm::vec3(0, 1, 0));
		shadowP = glm::perspective(shadowFov, 1.f, shadowZNear, shadowZFar);
		glUseProgram(const_Prog);
		setUniform(const_Prog, "modelMat", glm::mat4(1));
		setUniform(const_Prog, "projMat",  shadowP);
		setUniform(const_Prog, "viewMat",  shadowV);
		renderFunction();
		shadowMap.restoreVP();
	}

	glUseProgram(renderProg);
	setUniform(renderProg, "color",    glm::vec4(.8, .8, .8, 1));
	setUniform(renderProg, "modelMat", glm::mat4(1));
	setViewProj(renderProg);
	setUniform(renderProg, "lightPos",  lightPos);
	setUniform(renderProg, "iblCoeffs", iblCoeffs, 9);

	if (enableShadow) {
		setUniform(renderProg, "shadowEnabled", 1);
		shadowMap.bindDepth(renderProg, "shadowMap", 0);
		setUniform(renderProg, "shadowProj",    shadowP);
		setUniform(renderProg, "shadowZNear",   shadowZNear);
		setUniform(renderProg, "shadowZFar",    shadowZFar);
		setUniform(renderProg, "lightDir",      normalize(sceneCenter - lightPos));
		setUniform(renderProg, "cosLightFov",   cosf(shadowFov / 2));
		setUniform(renderProg, "shadowBiasedVP",
			glm::translate(glm::vec3(0.5)) * glm::scale(glm::vec3(0.5))
			* shadowP * shadowV);
	}
	else {
		setUniform(renderProg, "shadowEnabled", 0);
	}
	renderFunction();

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLineWidth(.2f);
	glUseProgram(const_Prog);
	setUniform(const_Prog, "color",    glm::vec4(0, 0, 0, .2));
	setUniform(const_Prog, "modelMat", glm::mat4(1));
	setViewProj(const_Prog);
	wireFunction();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}
