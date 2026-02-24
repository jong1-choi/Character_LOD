#include <iostream>
#include <filesystem>
#include "AnimView.h"
#include "GLTools.h"
#include "Motion.h"
#include "BVH.h"
#include "MotionFitting.h"
#include "KeyReduction.h"

using namespace std;

// Configuration
const std::string bvhDir = "../data/BVH_dance";

// Motion data
vector<BVH*>   bvhList;
vector<Motion> originalMotions;
vector<Motion> lodMotions;
vector<Motion> keyReducedMotions;

// Playback state
float        frameAccumulator  = 0.f;
unsigned int currentFrame      = 0;
int          currentMotionIndex = 4;

// UI & Window
AnimView*    animView = nullptr;
GLFWwindow*  gWindow  = nullptr;
InputState   g_input;


// --- GLFW Callbacks ---

static void keyCallback(GLFWwindow* w, int key, int /*scancode*/, int action, int /*mods*/) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        g_input.key = key;
        animView->handle(EVENT_KEYDOWN);
    }
}

static void mouseButtonCallback(GLFWwindow* w, int button, int action, int /*mods*/) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS)   animView->handle(EVENT_PUSH);
        if (action == GLFW_RELEASE) animView->handle(EVENT_RELEASE);
    }
}

static void cursorPosCallback(GLFWwindow* w, double x, double y) {
    g_input.mousePos = { (float)x, (float)y };
    if (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
        animView->handle(EVENT_DRAG);
}

static void scrollCallback(GLFWwindow* /*w*/, double /*x*/, double y) {
    g_input.scrollDelta = (float)y;
    animView->handle(EVENT_SCROLL);
}

static void dropCallback(GLFWwindow* /*w*/, int count, const char** paths) {
    g_input.droppedFiles.clear();
    for (int i = 0; i < count; i++)
        g_input.droppedFiles.push_back(paths[i]);
    animView->handle(EVENT_DND);
}

static void framebufferSizeCallback(GLFWwindow* /*w*/, int width, int height) {
    g_input.fbWidth  = width;
    g_input.fbHeight = height;
    if (animView) {
        animView->_w = (float)width;
        animView->_h = (float)height;
    }
}

static void windowSizeCallback(GLFWwindow* /*w*/, int width, int height) {
    g_input.winWidth  = width;
    g_input.winHeight = height;
}


// --- Motion Initialization ---

void LoadBVHFiles(const std::string& path, vector<BVH*>& outBvhList) {
    for (const auto& file : std::filesystem::directory_iterator(path)) {
        outBvhList.push_back(new BVH);
        outBvhList.back()->load(file.path().string().c_str());
    }
}

void InitMotions(const std::string& path, vector<BVH*>& outBvhList, vector<Motion>& outMotions, const int targetFps) {
    LoadBVHFiles(path, outBvhList);
    for (int i = 0; i < (int)outBvhList.size(); i++) {
        Motion temp;
        int k = 0;
        const int srcFps = int(1 / outBvhList[i]->interval);
        outMotions.push_back(temp);
        for (int j = 0; j < (int)outBvhList[i]->num_frame; j++) {
            if (srcFps != targetFps && j % (srcFps / targetFps) != 0) continue;
            Body body;
            outBvhList[i]->updatePose(j, body, 0.1f);
            outMotions[i].add(body);
            outMotions[i].bodies[k++].updateLink();
        }
        outMotions[i].totalFrames = (unsigned int)outMotions[i].bodies.size();
        outBvhList[i]->clear();
    }
}

void CopyMotions(const vector<Motion>& origin, vector<Motion>& copy) {
    std::copy(origin.begin(), origin.end(), std::back_inserter(copy));
}


// --- AnimView Callbacks ---

void onFrame(float dt) {
    constexpr float FRAME_DURATION = 1.f / 30.f;
    frameAccumulator += dt;
    if (frameAccumulator > FRAME_DURATION) {
        currentFrame++;
        frameAccumulator -= FRAME_DURATION;
    }
}

void onRender() {
    const size_t frameIndex = currentFrame % originalMotions[currentMotionIndex].bodies.size();

    originalMotions[currentMotionIndex].bodies[frameIndex].shapeRender(glm::vec3(  0, 0,    0), glm::vec4(0, 0, 0, 0));
    originalMotions[currentMotionIndex].bodies[frameIndex].shapeRender(glm::vec3(-40, 0, -300), glm::vec4(0, 0, 0, 0), 0.8f);
    originalMotions[currentMotionIndex].bodies[frameIndex].shapeRender(glm::vec3( 40, 0, -800), glm::vec4(0, 0, 0, 0), 1.f);

    lodMotions[currentMotionIndex].bodies[frameIndex].shapeRender(glm::vec3(-10, 0,    0), glm::vec4(0, 0, 0, 0));
    lodMotions[currentMotionIndex].bodies[frameIndex].shapeRender(glm::vec3(-50, 0, -300), glm::vec4(0, 0, 0, 0), 0.8f);
    lodMotions[currentMotionIndex].bodies[frameIndex].shapeRender(glm::vec3( 30, 0, -800), glm::vec4(0, 0, 0, 0), 1.f);

    keyReducedMotions[currentMotionIndex].bodies[frameIndex].shapeRender(glm::vec3(10, 0,    0), glm::vec4(0, 0, 0, 0));
    keyReducedMotions[currentMotionIndex].bodies[frameIndex].shapeRender(glm::vec3(-30, 0, -300), glm::vec4(0, 0, 0, 0), 0.8f);
    keyReducedMotions[currentMotionIndex].bodies[frameIndex].shapeRender(glm::vec3( 50, 0, -800), glm::vec4(0, 0, 0, 0), 1.f);
}

void onInit() {
    currentFrame = 1;
}

void onControl(char c) {
    if (c == '[') currentMotionIndex--;
    else if (c == ']') currentMotionIndex++;
    currentFrame = 0;
    std::cout << currentMotionIndex << " " << currentFrame << std::endl;
}


// --- Entry Point ---

int main(int argc, const char* argv[]) {

    // Init GLFW
    if (!glfwInit()) {
        std::cerr << "[ERROR] glfwInit() failed\n";
        return -1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    constexpr int WIN_W = 1280;
    constexpr int WIN_H = 720;
    gWindow = glfwCreateWindow(WIN_W, WIN_H, "simulation", nullptr, nullptr);
    if (!gWindow) {
        std::cerr << "[ERROR] glfwCreateWindow() failed\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(gWindow);
    glfwSwapInterval(1);  // vsync

    // Query initial window/framebuffer size
    glfwGetWindowSize(gWindow, &g_input.winWidth, &g_input.winHeight);
    glfwGetFramebufferSize(gWindow, &g_input.fbWidth, &g_input.fbHeight);

    // Init GLEW (load OpenGL functions)
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cerr << "[ERROR] glewInit() failed\n";
        glfwTerminate();
        return -1;
    }

    // Load and process motion data
    InitMotions(bvhDir, bvhList, originalMotions, 30);
    CopyMotions(originalMotions, lodMotions);
    CopyMotions(originalMotions, keyReducedMotions);

    for (int i = 0; i < originalMotions.size(); i++) {
        lodMotions[i].simplifySkeleton(2);
        keyReducedMotions[i].simplifySkeleton(2);
    }
    for (int i = 0; i < originalMotions.size(); i++) {
        FitMotion(2, originalMotions[i], lodMotions[i]);
        FitMotion(2, originalMotions[i], keyReducedMotions[i]);
    }
    ReduceKeyframes(keyReducedMotions[0], 2, 10);

    // Create AnimView and bind callbacks
    animView = new AnimView(0, 0, (float)WIN_W, (float)WIN_H);
    animView->initFunction    = onInit;
    animView->controlFunction = onControl;
    animView->renderFunction  = onRender;
    animView->frameFunction   = onFrame;

    // Register GLFW callbacks
    glfwSetKeyCallback            (gWindow, keyCallback);
    glfwSetMouseButtonCallback    (gWindow, mouseButtonCallback);
    glfwSetCursorPosCallback      (gWindow, cursorPosCallback);
    glfwSetScrollCallback         (gWindow, scrollCallback);
    glfwSetDropCallback           (gWindow, dropCallback);
    glfwSetFramebufferSizeCallback(gWindow, framebufferSizeCallback);
    glfwSetWindowSizeCallback     (gWindow, windowSizeCallback);

    // Render loop
    float lastTime = (float)glfwGetTime();
    while (!glfwWindowShouldClose(gWindow)) {
        const float t  = (float)glfwGetTime();
        const float dt = t - lastTime;
        lastTime = t;

        if (animView->animating)
            animView->update(dt);

        glViewport(0, 0, g_input.fbWidth, g_input.fbHeight);
        animView->drawGL();

        glfwSwapBuffers(gWindow);
        glfwPollEvents();
    }

    delete animView;
    glfwDestroyWindow(gWindow);
    glfwTerminate();
    return 0;
}
