#pragma once
#include "motion.hpp"
#include <sciplot/sciplot.hpp>

void DrawDot() {

}

void DrawLine() {

}

void CompressFrame(Motion& motion, int level, int window) {
    vector<int> links;
    vector<float> distance;
    vector<float> direction;
    vector<float> dirxdist;
    vector<int> range;
    vector<pair<int, float>> beats;
    for (int i = 0; i < motion.frameNum; i++) {
        range.push_back(i);
    }
    for (auto& l : motion.bodies[0].links) {
        if (level == l.level) links.push_back(l.index);
    }
    for (auto& index : links) {
        for (int i = 0; i < motion.frameNum; i++) {
            float dist(0);
            vec3 v1(0), v2(0);
            vec3 beforeVec(0), originVec(0), afterVec(0), beforeDir(0), afterDir(0);
            float beforeDiff(0), afterDiff(0);

            for (int j = 1; j <= window; j++) {
                if ((i + j) >= motion.frameNum || (i - j) < 0) continue;
                beforeVec = normalize(motion.bodies[i - j].links[index].getDistance());
                originVec = normalize(motion.bodies[i].links[index].getDistance());
                afterVec = normalize(motion.bodies[i + j].links[index].getDistance());
                beforeDir = cross(beforeVec, originVec);
                afterDir = cross(originVec, afterVec);
                beforeDiff = length(originVec - beforeVec);
                //afterDiff = length(originVec - afterVec);

                dist += beforeDiff + afterDiff;
                v1 += normalize(beforeDir);
                v2 += normalize(afterDir);
            }


            float dir = dot(normalize(v1), normalize(v2));
            if ((i + window) >= motion.frameNum || (i - window) < 0) {
                dir = 0;
                dist = 0;
            }
            distance.push_back(dist);
            direction.push_back(dir);
            dirxdist.push_back(dist * dir);
        }

        vector<int> dotIndex;
        vector<float> dotValue;
        dotIndex.push_back(0);
        dotValue.push_back(0);
        for (int i = 1; i < motion.frameNum - 1; i++) {
            if (dirxdist[i] < -1 && dirxdist[i] < dirxdist[i - 1] && dirxdist[i] < dirxdist[i + 1]) {
                dotIndex.push_back(i);
                dotValue.push_back(dirxdist[i]);
            }
        }
        dotIndex.push_back(motion.frameNum - 1);
        dotValue.push_back(0);

        for (int i = 0; i < dotIndex.size(); i++) {
            beats.push_back(make_pair(dotIndex[i], dotValue[i]));
        }

        //sciplot::Plot2D plot;
        //plot.legend()
        //    .atOutsideBottom()
        //    .fontSize(10)
        //    .displayHorizontal();
        //
        //auto curve1 = plot.drawCurve(range, distance).label("distance");
        //auto curve2 = plot.drawCurve(range, direction).label("direction");
        //auto curve3 = plot.drawCurve(range, dirxdist).label("distance x direction");
        //auto dot = plot.drawPoints(dotIndex, dotValue);
        //
        //_putenv("PATH=C:\\\gnuplot\\bin;%PATH%");
        //_popen("gnuplot", "w");
        //
        //sciplot::Figure fig = { {plot} };
        //sciplot::Canvas canvas = { {fig} };
        //canvas.size(1280, 720);
        //canvas.show();

        distance.clear();
        direction.clear();
        dirxdist.clear();
        dotIndex.clear();
        dotValue.clear();
    }

    sort(beats.begin(), beats.end());

    vector<int> index;
    vector<float> value;
    vector<int> finalBeats;
    vector<int> zero;
    for (auto& b : beats) {
        index.push_back(b.first);
        value.push_back(b.second);
    }

    finalBeats.push_back(0);
    zero.push_back(0);
    for (int i = 1; i < beats.size(); i++) {
        int tempI = beats[i].first;
        float maxV = beats[i].second;
        int maxI = tempI;

        while (1) {
            if (beats[i + 1].first <= tempI + window/2 && i < beats.size() - 1) {
                if (beats[i + 1].second < maxV) {
                    maxI = beats[i + 1].first;
                    maxV = beats[i + 1].second;
                }
                i++;
            }
            else break;
        }

        if (maxI - finalBeats.back() < window / 2) continue;
        else if (maxI - finalBeats.back() > window * 3) {
            int extraKey = (maxI - finalBeats.back()) / (window * 3);
            int interval = (maxI - finalBeats.back()) / (extraKey + 1);
            int startIndex = finalBeats.back();
            for (int j = 1; j < extraKey + 1; j++) {
                finalBeats.push_back(startIndex + j * interval);
                zero.push_back(0);
            }
        }

        finalBeats.push_back(maxI);
        zero.push_back(0);
    }

    std::cout <<  finalBeats.size() / float(motion.frameNum) << std::endl;

    sciplot::Plot2D plot;
    plot.legend()
    	.atOutsideBottom()
    	.fontSize(10)
    	.displayHorizontal();
    
    plot.drawPoints(index, value);
    plot.drawPoints(finalBeats, zero);
    
    _putenv("PATH=C:\\\gnuplot\\bin;%PATH%");
    _popen("gnuplot", "w");
    
    sciplot::Figure fig = { {plot} };
    sciplot::Canvas canvas = { {fig} };
    canvas.size(1280, 720);
    canvas.show();

    for (auto& index : links) {
        if (index == 0) continue;
        for (int i = 0; i < finalBeats.size() - 1; i++) {
            quat q1 = motion.bodies[finalBeats[i]].links[index].q;
            quat q2 = motion.bodies[finalBeats[i + 1]].links[index].q;
            if (dot(q1, q2) < 0) q2 = -q2;
            for (float j = finalBeats[i]; j < finalBeats[i + 1]; j++) {
                motion.bodies[j].links[index].q = mix(q1, q2, (j - finalBeats[i]) / (finalBeats[i + 1] - finalBeats[i]));
            }
        }
    }

    for (auto& body : motion.bodies) {
        body.updateLink();
    }
}
