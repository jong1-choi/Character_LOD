#include "MotionFitting.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

using namespace glm;
using namespace std;


void FitMotionDirect(unsigned int level, const Motion& origin, Motion& fitted, const vector<int>& targets) {
    int frame = 0;
    for (auto& body : fitted.bodies) {
        for (const auto& target : targets) {
            const int parent = body.getAncestors(target, level)[0];

            const vec3 targetPos   = origin.bodies[frame].links[target].getWorldPos();
            const vec3 jointPos    = body.links[parent].getWorldPos();
            const vec3 effectorPos = body.links[target].getWorldPos();

            if (length(targetPos - effectorPos) < 0.001f) continue;

            vec3 beforeV = normalize(effectorPos - jointPos);
            vec3 afterV  = normalize(targetPos  - jointPos);

            beforeV = inverse(body.links[parent].parentWorldRot) * beforeV;
            afterV  = inverse(body.links[parent].parentWorldRot) * afterV;

            const vec3  axis  = normalize(cross(beforeV, afterV));
            const float theta = acos(dot(beforeV, afterV));
            const quat  rot   = normalize(quat(cos(theta / 2.f), sin(theta / 2.f) * axis));

            body.links[parent].applyLocalRotation(rot);
            body.updateLink();
        }
        frame++;
    }
}

static float ComputeReachError(const Body& body, const int start, const int end, const vec3 target) {
    float totalLen = 0.f;
    const float targetLen = length(target - body.links[end].getWorldPos());
    int index = start;
    while (true) {
        if (body.links[index].index == end) break;
        totalLen += length(body.links[index].boneOffset);
        index = body.links[index].parentIndex;
    }
    return targetLen - totalLen;
}

void FitMotionIK(unsigned int level, Motion& origin, Motion& fitted, const vector<int>& targets) {
    constexpr float learningRate = 0.01f;
    constexpr float extra        = 0.001f;
    const vec3 axes[] = { {1,0,0}, {0,1,0}, {0,0,1} };

    int frame = 0;
    for (auto& body : fitted.bodies) {
        int targetIdx = -1;
        for (const auto& target : targets) {
            targetIdx++;
            vector<int> joints;
            if (targetIdx == 0) joints = body.getAncestors(target, level);
            else                joints = body.getAncestors(target, level, targets[targetIdx - 1]);

            if (joints.size() == 0) continue;

            const vec3  targetPos = origin.bodies[frame].links[target].getWorldPos();
            const float diff      = ComputeReachError(body, target, joints.back(), targetPos);

            vec3 d = targetPos - body.links[target].getWorldPos();
            if (length(d) < abs(diff) + extra) continue;

            const vec3 modifiedTarget =
                (1 - diff / length(targetPos - body.links[joints.back()].getWorldPos()))
                * (targetPos - body.links[joints.back()].getWorldPos())
                + body.links[joints.back()].getWorldPos();

            Eigen::MatrixXf J = Eigen::MatrixXf(3, 3);
            Eigen::MatrixXf b = Eigen::MatrixXf(3, 1);

            for (int iter = 0; iter < 1000; iter++) {
                for (const auto& joint : joints) {
                    d = modifiedTarget - body.links[target].getWorldPos();
                    if (length(targetPos - body.links[target].getWorldPos()) < abs(diff) + extra) continue;

                    const vec3 p = body.links[target].getWorldPos() - body.links[joint].getWorldPos();
                    for (int j = 0; j < 3; j++) {
                        const vec3 v = cross(axes[j], p);
                        J.col(j) << v.x, v.y, v.z;
                    }
                    b << d.x, d.y, d.z;
                }
                auto solver = J.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
                solver.setThreshold(0.01f);
                const auto x = solver.solve(b);
                for (int ji = 0; ji < (int)joints.size(); ji++) {
                    for (int j = 0; j < 3; j++) {
                        body.links[joints[ji]].localRot =
                            quat(cos(learningRate * x(ji * 3 + j) / 2),
                                 sin(learningRate * x(ji * 3 + j) / 2) * axes[j])
                            * body.links[joints[ji]].localRot;
                    }
                }
                body.updateLink();
            }
        }
        frame++;
    }
}

void FitMotion(unsigned int level, Motion& origin, Motion& fitted) {
    if (level == 1) {
        const vector<int> spine    = { 3 };
        const vector<int> leftArm  = { 8, 9 };
        const vector<int> rightArm = { 32, 33 };
        const vector<int> leftLeg  = { 55, 56, 60 };
        const vector<int> rightLeg = { 61 };
        FitMotionDirect(level, origin, fitted, spine);
        FitMotionDirect(level, origin, fitted, leftArm);
        FitMotionDirect(level, origin, fitted, rightArm);
        FitMotionDirect(level, origin, fitted, leftLeg);
        FitMotionDirect(level, origin, fitted, rightLeg);
    }
    if (level == 2) {
        const vector<int> leftArm  = { 9 };
        const vector<int> rightArm = { 33 };
        const vector<int> leftLeg  = { 56 };
        const vector<int> rightLeg = { 61 };
        FitMotionDirect(level, origin, fitted, leftArm);
        FitMotionDirect(level, origin, fitted, rightArm);
        FitMotionDirect(level, origin, fitted, leftLeg);
        FitMotionDirect(level, origin, fitted, rightLeg);
    }
}
