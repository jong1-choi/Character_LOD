#include "KeyReduction.h"
#include <sciplot/sciplot.hpp>

using namespace glm;
using namespace std;


void ReduceKeyframes(Motion& motion, unsigned int level, int windowSize) {
    vector<int>   levelLinks;
    vector<float> distance;
    vector<float> direction;
    vector<float> dirxdist;
    vector<int>   frameRange;
    vector<pair<int, float>> keyFrameCandidates;

    for (int i = 0; i < (int)motion.totalFrames; i++) {
        frameRange.push_back(i);
    }
    for (const auto& link : motion.bodies[0].links) {
        if (level == link.level) levelLinks.push_back(link.index);
    }

    for (const auto& linkIndex : levelLinks) {
        for (int i = 0; i < (int)motion.totalFrames; i++) {
            float dist = 0.f;
            vec3  v1(0), v2(0);

            for (int j = 1; j <= windowSize; j++) {
                if ((i + j) >= (int)motion.totalFrames || (i - j) < 0) continue;
                const vec3 beforeVec = normalize(motion.bodies[i - j].links[linkIndex].getBoneVector());
                const vec3 originVec = normalize(motion.bodies[i    ].links[linkIndex].getBoneVector());
                const vec3 afterVec  = normalize(motion.bodies[i + j].links[linkIndex].getBoneVector());
                const vec3 beforeDir = cross(beforeVec, originVec);
                const vec3 afterDir  = cross(originVec, afterVec);
                const float beforeDiff = length(originVec - beforeVec);

                dist += beforeDiff;
                v1 += normalize(beforeDir);
                v2 += normalize(afterDir);
            }

            float dir = dot(normalize(v1), normalize(v2));
            if ((i + windowSize) >= (int)motion.totalFrames || (i - windowSize) < 0) {
                dir  = 0.f;
                dist = 0.f;
            }
            distance.push_back(dist);
            direction.push_back(dir);
            dirxdist.push_back(dist * dir);
        }

        vector<int>   localMinFrames;
        vector<float> localMinValues;
        localMinFrames.push_back(0);
        localMinValues.push_back(0.f);
        for (int i = 1; i < (int)motion.totalFrames - 1; i++) {
            if (dirxdist[i] < -1 && dirxdist[i] < dirxdist[i - 1] && dirxdist[i] < dirxdist[i + 1]) {
                localMinFrames.push_back(i);
                localMinValues.push_back(dirxdist[i]);
            }
        }
        localMinFrames.push_back(motion.totalFrames - 1);
        localMinValues.push_back(0.f);

        for (int i = 0; i < (int)localMinFrames.size(); i++) {
            keyFrameCandidates.push_back(make_pair(localMinFrames[i], localMinValues[i]));
        }

        distance.clear();
        direction.clear();
        dirxdist.clear();
        localMinFrames.clear();
        localMinValues.clear();
    }

    sort(keyFrameCandidates.begin(), keyFrameCandidates.end());

    vector<int>   candidateFrames;
    vector<float> candidateValues;
    for (const auto& kf : keyFrameCandidates) {
        candidateFrames.push_back(kf.first);
        candidateValues.push_back(kf.second);
    }

    vector<int> selectedKeyFrames;
    vector<int> zeroLine;
    selectedKeyFrames.push_back(0);
    zeroLine.push_back(0);

    for (int i = 1; i < (int)keyFrameCandidates.size(); i++) {
        int   tempI = keyFrameCandidates[i].first;
        float maxV  = keyFrameCandidates[i].second;
        int   maxI  = tempI;

        while (true) {
            if (i >= (int)keyFrameCandidates.size() - 1)
                break;

            if (keyFrameCandidates[i + 1].first <= tempI + windowSize / 2 && i < (int)keyFrameCandidates.size() - 1) {
                if (keyFrameCandidates[i + 1].second < maxV) {
                    maxI = keyFrameCandidates[i + 1].first;
                    maxV = keyFrameCandidates[i + 1].second;
                }
                i++;
            }
            else break;
        }

        if (maxI - selectedKeyFrames.back() < windowSize / 2) continue;
        else if (maxI - selectedKeyFrames.back() > windowSize * 3) {
            const int extraCount  = (maxI - selectedKeyFrames.back()) / (windowSize * 3);
            const int interval    = (maxI - selectedKeyFrames.back()) / (extraCount + 1);
            const int startIndex  = selectedKeyFrames.back();
            for (int j = 1; j < extraCount + 1; j++) {
                selectedKeyFrames.push_back(startIndex + j * interval);
                zeroLine.push_back(0);
            }
        }

        selectedKeyFrames.push_back(maxI);
        zeroLine.push_back(0);
    }

    std::cout << selectedKeyFrames.size() / float(motion.totalFrames) << std::endl;

    sciplot::Plot2D plot;
    plot.legend()
        .atOutsideBottom()
        .fontSize(10)
        .displayHorizontal();
    plot.drawPoints(candidateFrames, candidateValues);
    plot.drawPoints(selectedKeyFrames, zeroLine);

    sciplot::Figure fig    = { {plot} };
    sciplot::Canvas canvas = { {fig} };
    canvas.size(1280, 720);
    canvas.show();

    for (const auto& linkIndex : levelLinks) {
        if (linkIndex == 0) continue;
        for (int i = 0; i < (int)selectedKeyFrames.size() - 1; i++) {
            quat q1 = motion.bodies[selectedKeyFrames[i    ]].links[linkIndex].localRot;
            quat q2 = motion.bodies[selectedKeyFrames[i + 1]].links[linkIndex].localRot;
            if (dot(q1, q2) < 0) q2 = -q2;
            for (int j = selectedKeyFrames[i]; j < selectedKeyFrames[i + 1]; j++) {
                const float t = (float)(j - selectedKeyFrames[i]) / (float)(selectedKeyFrames[i + 1] - selectedKeyFrames[i]);
                motion.bodies[j].links[linkIndex].localRot = mix(q1, q2, t);
            }
        }
    }

    for (auto& body : motion.bodies) {
        body.updateLink();
    }
}
