#pragma once
#include <Eigen/SVD>
#include <Eigen/Dense>
#include "motion.hpp"

void MotionFittingDirect(const int level, const Motion& origin, Motion& Fitted, const vector<int>& targets) {
    int frame = 0;
    for (auto& body : Fitted.bodies) {
        for (auto& target : targets) {
            vector<int> joints;

            int parent = body.getAncestors(target, level)[0];

            vec3 targetP = origin.bodies[frame].links[target].getPos();
            vec3 jointP = body.links[parent].getPos();
            vec3 effectorP = body.links[target].getPos();

            if (length(targetP - effectorP) < 0.001) continue;

            vec3 beforeV = normalize(effectorP - jointP);
            vec3 afterV = normalize(targetP - jointP);

            beforeV = inverse(body.links[parent].parentGlobalQ) * beforeV;
            afterV = inverse(body.links[parent].parentGlobalQ) * afterV;

            vec3 axis = normalize(cross(beforeV, afterV));
            float theta = acos(dot(beforeV, afterV));

            quat q = normalize(quat(cos(theta / 2.f), sin(theta / 2.f) * axis));

            body.links[parent].rotate(q);
            body.updateLink();
        }
        frame++;
    }
}

const float ReturnTargetDiff(const Body& body, const int start, const int end, const vec3 target) {
    float totalLen(0), targetLen(0);
    int index = start;
    targetLen = length(target - body.links[end].getPos());

    while (1) {
        if (body.links[index].index == end) break;
        else totalLen += length(body.links[index].l);
        index = body.links[index].parentIndex;
    }

    return targetLen - totalLen;
}

void MotionFittingIK(const int level, Motion& origin, Motion& Fitted, const vector<int>& targets) {
    int frame = 0;
    float learningRate = 0.01;
    for (auto& body : Fitted.bodies) {
        int i = -1;
        for (auto& target : targets) {
            i++;
            vector<int> joints;

            if (i == 0) joints = body.getAncestors(target, level);
            else joints = body.getAncestors(target, level, targets[i - 1]);

            if (joints.size() == 0) continue;

            float diff(0), extra(0.001);
            vec3 targetP = origin.bodies[frame].links[target].getPos();

            diff = ReturnTargetDiff(body, target, joints.back(), targetP);

            vec3 d = targetP - body.links[target].getPos();
            if (length(d) < abs(diff) + extra) continue;

            vec3 modifiedTarget = (1 - diff / length(targetP - body.links[joints.back()].getPos())) * (targetP - body.links[joints.back()].getPos()) + body.links[joints.back()].getPos();
            vec3 axes[] = { {1,0,0}, {0,1,0}, {0,0,1} };
            Eigen::MatrixXf J = Eigen::MatrixXf(3, 3);
            Eigen::MatrixXf b = Eigen::MatrixXf(3, 1);


            for (int iter = 0; iter < 1000; iter++) {
                for (auto& joint : joints) {
                    d = modifiedTarget - body.links[target].getPos();
                    if (length(targetP - body.links[target].getPos()) < abs(diff) + extra) continue;

                    vec3 p = body.links[target].getPos() - body.links[joint].getPos();
                    for (int j = 0; j < 3; j++) {
                        vec3 axis = body.links[joint].parentGlobalQ * axes[j];
                        vec3 v = cross(axes[j], p);
                        J.col(j) << v.x, v.y, v.z;
                    }
                    b << d.x, d.y, d.z;
                }
                auto solver = J.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
                solver.setThreshold(0.01);
                auto x = solver.solve(b);
                for (int i = 0; i < joints.size(); i++) {
                    for (int j = 0; j < 3; j++) {

                        body.links[joints[i]].q = (quat(cos(learningRate * x(i * 3 + j) / 2), sin(learningRate * x(i * 3 + j) / 2) * axes[j])) * body.links[joints[i]].q;
                    }
                }
                body.updateLink();
            }
        }
        frame++;
    }
}
void FitMotion(const int level, Motion& origin, Motion& Fitted) {
    if (level == 1) {
        vector<int> spine;
        vector<int> leftArm;
        vector<int> rightArm;
        vector<int> leftLeg;
        vector<int> rightLeg;
        spine.push_back(3);
        leftArm.push_back(8);
        leftArm.push_back(9);
        rightArm.push_back(32);
        rightArm.push_back(33);
        leftLeg.push_back(55);
        leftLeg.push_back(56);
        leftLeg.push_back(60);
        rightLeg.push_back(61);
        MotionFittingDirect(level, origin, Fitted, spine);
        MotionFittingDirect(level, origin, Fitted, leftArm);
        MotionFittingDirect(level, origin, Fitted, rightArm);
        MotionFittingDirect(level, origin, Fitted, leftLeg);
        MotionFittingDirect(level, origin, Fitted, rightLeg);
    }
    if (level == 2) {
        vector<int> leftArm;
        vector<int> rightArm;
        vector<int> leftLeg;
        vector<int> rightLeg;
        leftArm.push_back(9);
        rightArm.push_back(33);
        leftLeg.push_back(56);
        rightLeg.push_back(61);
        MotionFittingDirect(level, origin, Fitted, leftArm);
        MotionFittingDirect(level, origin, Fitted, rightArm);
        MotionFittingDirect(level, origin, Fitted, leftLeg);
        MotionFittingDirect(level, origin, Fitted, rightLeg);
    }
}