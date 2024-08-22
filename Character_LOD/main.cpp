#define GL_SILENCE_DEPRECATION

#include <iostream>
#include <filesystem>
#include <JGL/JGL_Window.hpp>
#include <thread>
#include "AnimView.hpp"
#include "GLTools.hpp"
#include "motion.hpp"
#include "readBVH.hpp"
#include "MotionFitting.hpp"
#include "KeyReduction.hpp"
 
float frameTime = 0;
unsigned int frameNum = 0;
unsigned int frameNum2 = 0;
unsigned int frameNum3 = 0;
AnimView* animView;
vector< BVH * > bvh;
vector< Motion > motions;
vector< Motion > copyMotions_1;
vector< Motion > copyMotions_11;
vector< Motion > copyMotions_2;
vector< Motion > copyMotions_22;
std::string path = "C:\\Users\\cjw1464\\source\\repos\\Character_LOD\\data\\BVH_dance";

int motionNum = 4;

void GetFile(std::string path, vector< BVH * > &bvh) {
    for( const auto &file : std::filesystem::directory_iterator(path)){
        bvh.push_back(new BVH);
        bvh.back()->Load( file.path().string().c_str() );
    }
}

void Init(std::string path, vector< BVH * > &bvh, vector<Motion> &motions, int f) {
    GetFile(path, bvh);
    for( int i = 0; i < bvh.size(); i++ ) {
        Motion temp;
        int k = 0;
        int fps = int( 1 / bvh[i]->interval );
        motions.push_back(temp);
        for(int j = 0; j < bvh[i]->num_frame; j++) {
            if(fps != f && j % (fps/f) != 0) continue;
            Body body;
            bvh[i]->UpdatePose(j,body,0.1);
            motions[i].add(body);
            motions[i].bodies[k++].updateLink();
        }
        motions[i].frameNum = motions[i].bodies.size();
        bvh[i]->Clear();
    }
}

void CopyMotion(const vector<Motion>& origin, vector<Motion>& copy) {
    std::copy(origin.begin(), origin.end(), std::back_inserter(copy));
}

void frame(float dt) {
    frameTime += dt;
    if(frameTime > 1/30.f){
        frameNum ++;
        frameTime -= 1/30.f;
    }
    frameNum2 = (frameNum / 2) * 2;
    frameNum3 = (frameNum / 3) * 3;
}

void render() {
    motions[motionNum].bodies[frameNum % motions[motionNum].bodies.size()].shapeRender(glm::vec3(0, 0, -0), glm::vec4(0, 0, 0, 0));
    motions[motionNum].bodies[frameNum % motions[motionNum].bodies.size()].shapeRender(glm::vec3(0-40, 0, -300), glm::vec4(0, 0, 0, 0), 0.8f);
    motions[motionNum].bodies[frameNum % motions[motionNum].bodies.size()].shapeRender(glm::vec3(0+40, 0, -800), glm::vec4(0, 0, 0, 0), 1.f);

    copyMotions_2[motionNum].bodies[frameNum % motions[motionNum].bodies.size()].shapeRender(glm::vec3(-10, 0, -0), glm::vec4(0, 0, 0, 0));
    copyMotions_2[motionNum].bodies[frameNum % motions[motionNum].bodies.size()].shapeRender(glm::vec3(-10 - 40, 0, -300), glm::vec4(0, 0, 0, 0), 0.8f);
    copyMotions_2[motionNum].bodies[frameNum % motions[motionNum].bodies.size()].shapeRender(glm::vec3(-10 + 40, 0, -800), glm::vec4(0, 0, 0, 0), 1.f);
    
    copyMotions_22[motionNum].bodies[frameNum % motions[motionNum].bodies.size()].shapeRender(glm::vec3(10, 0, -0), glm::vec4(0, 0, 0, 0));
    copyMotions_22[motionNum].bodies[frameNum % motions[motionNum].bodies.size()].shapeRender(glm::vec3(10 - 40, 0, -300), glm::vec4(0, 0, 0, 0), 0.8f);
    copyMotions_22[motionNum].bodies[frameNum % motions[motionNum].bodies.size()].shapeRender(glm::vec3(10 + 40, 0, -800), glm::vec4(0, 0, 0, 0), 1.f);
}

void initFrame() {
    frameNum = 1;
}

void controlFrame(char c) {
    if(c == '[') motionNum--;
    else if(c == ']') motionNum++;
    frameNum = 0;
    //if (c == '[') frameNum--;
    //else if (c == ']') frameNum++;
    //if(frameNum < 0) frameNum = 0;
    std::cout << motionNum << " " << frameNum << std::endl;
}


int main(int argc, const char * argv[]) {
    
    Init(path, bvh, motions, 30);
    CopyMotion(motions, copyMotions_1);
    CopyMotion(motions, copyMotions_11);
    CopyMotion(motions, copyMotions_2);
    CopyMotion(motions, copyMotions_22);

    for (int i = 0; i < motions.size(); i++) {
        copyMotions_1[i].SimplifySkeleton(1);
        copyMotions_11[i].SimplifySkeleton(1);
        copyMotions_2[i].SimplifySkeleton(2);
        copyMotions_22[i].SimplifySkeleton(2);
    }

    for (int i = 0; i < motions.size(); i++) {
        FitMotion(1, motions[i], copyMotions_1[i]);
        FitMotion(1, motions[i], copyMotions_11[i]);
        FitMotion(2, motions[i], copyMotions_2[i]);
        FitMotion(2, motions[i], copyMotions_22[i]);
    }

    //for (int i = 0; i < motions.size(); i++) {
    //    CompressFrame(copyMotions_11[i], 1, 10);
    //    CompressFrame(copyMotions_22[i], 2, 10);
    //}
    CompressFrame(copyMotions_22[0], 2, 10);

    JGL::Window* window = new JGL::Window(1280, 720, "simulation");
    window->alignment(JGL::ALIGN_ALL);
    animView = new AnimView(0, 0, 1280, 720);
    animView->initFunction = initFrame;
    animView->controlFunction = controlFrame;
    animView->renderFunction = render;
    animView->frameFunction = frame;
    window->show();
    JGL::_JGL::run();
    
    return 0;
}
