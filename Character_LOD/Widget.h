#ifndef Widget_h
#define Widget_h

#include <glm/glm.hpp>
#include <string>
#include <vector>

// Event types replacing JGL::EVENT_*
enum Event {
    EVENT_PUSH    = 1,
    EVENT_DRAG    = 2,
    EVENT_RELEASE = 3,
    EVENT_SCROLL  = 4,
    EVENT_KEYDOWN = 5,
    EVENT_DND     = 6
};

// Global input state: written by GLFW callbacks, read inside handle()
struct InputState {
    glm::vec2 mousePos    = { 0.f, 0.f };
    int       key         = 0;
    float     scrollDelta = 0.f;
    int       fbWidth     = 0;
    int       fbHeight    = 0;
    int       winWidth    = 0;
    int       winHeight   = 0;
    std::vector<std::string> droppedFiles;
};

// Defined in main.cpp, used via extern everywhere else
extern InputState g_input;

// Base widget class replacing JGL::Widget
struct Widget {
    float       _x, _y, _w, _h;
    std::string _name;

    Widget(float x, float y, float ww, float hh, const std::string& name = "")
        : _x(x), _y(y), _w(ww), _h(hh), _name(name) {}

    float     w() const { return _w; }
    float     h() const { return _h; }
    glm::vec2 abs_pos() const { return glm::vec2(_x, _y); }

    // HiDPI ratio: framebuffer pixels / window pixels
    float pxRatio() const {
        return (g_input.winWidth > 0) ? (float)g_input.fbWidth / (float)g_input.winWidth : 1.f;
    }

    struct WindowSize { int w, h; };
    WindowSize getWindowSize() const { return { g_input.winWidth, g_input.winHeight }; }

    // No-ops: GLFW loop renders every frame
    void redraw() {}
    void animate() {}

    virtual bool handle(int e) { return false; }
    virtual void drawGL() {}
    virtual ~Widget() {}
};

#endif /* Widget_h */
