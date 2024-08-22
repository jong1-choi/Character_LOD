#ifndef motion_hpp
#define motion_hpp

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "GLTools.hpp"

using namespace glm;
using namespace std;

const int LEVEL_1[10] = { 0, 2, 7, 8, 31, 32, 54, 55, 59, 60 };
const int LEVEL_2[5] = { 0, 7, 31, 54, 59 };

struct Link {
    glm::vec3 parentGlobalP = glm::vec3(0);
    glm::quat parentGlobalQ = glm::quat(1, 0, 0, 0);
    glm::vec3 l = glm::vec3(0);
    glm::quat q = glm::quat(1, 0, 0, 0);
    glm::vec3 v = glm::vec3(0);

    bool isFixed = false;
    bool isHand = false;
    bool isEnd = false;
    bool isLevelRoot = false;
    int level = 0;
    int index = -1;
    int parentIndex = -1;
    int childIndex = -1;

    //void render() const {
    //    drawSphere( getPos(), 0.2);
    //    drawCylinder( getPos(), parentGlobalP, 0.1 );
    //}
    void shapeRender(glm::vec3 offset, glm::vec4 color = glm::vec4(1, 1, 1, 0), float size = 1, float multiple = 2) const {
        drawCylinder(multiple * (getPos() + offset), multiple * (parentGlobalP + offset), size, color);
        if (!isFixed) drawSphere(multiple * (getPos() + offset), 0.5, vec4(1, 0, 0, 0));
    }
    glm::vec3 getPos() const {
        return parentGlobalQ * l + parentGlobalP;
    }
    glm::vec3 getPos(float multiplier) const {
        if(parentIndex == -1) return l * q;
        return parentGlobalQ * l * multiplier + parentGlobalP;
    }
    glm::vec3 getDistance() const {
        return q * l;
    }
    glm::vec3 getNearP(glm::vec3 dir, float scale = 1) const {
        return getOri() * dir * scale * 0.2f + getPos();
    }
    glm::quat getOri() const {
        return parentGlobalQ * q;
    }
    void rotate( const glm::quat& rot ) {
        q = rot * q;
    }
    void updatePose( const glm::vec3& pos, const glm::quat& ori ) {
        parentGlobalP = pos;
        parentGlobalQ = ori;
    }
    Link( int parI, int chiN, const glm::vec3& ll, const glm::quat& qq, const glm::quat& pq, bool end, bool h, int i) : parentIndex(parI), childIndex(chiN), l(ll), q(qq), parentGlobalQ(pq), isEnd(end), isHand(h), index(i){
        SetLevelRoot();
    }
    void SetLevelRoot() {
        if (index == 1 || index == 6 || index == 30 || index == 54 || index == 59) isLevelRoot = true;
    }
};


struct Body {
    std::vector<Link> links;
    glm::vec3 globalP = glm::vec3(0);
    glm::quat globalQ = glm::quat(1, 0, 0, 0);
    
    int test = 0;

    ~Body() {
        clear();
    }
    void clear() {
        links.clear();
        vector<Link>().swap(links);

        globalP = glm::vec3(0);
        globalQ = glm::quat(1, 0, 0, 0);
    }
    void add( int parI, int chiI, const glm::vec3& ll, const glm::quat& qq, const glm::quat& pq, bool end, bool h, int i ) {
            links.push_back(Link(parI, chiI, ll, qq, pq, end, h, i));
    }
    void shapeRender(glm::vec3 offset = glm::vec3(0), glm::vec4 color = glm::vec4(1,1,1,0), float size = 0.4) const {
        for (auto& l : links) {
            if(l.parentIndex >= 0)
                l.shapeRender(offset, color, size);
        }
    }

    const std::vector<int> getAncestors(int end) {
        std::vector<int> ret;
        int index = end;
        bool endCheck = 1;

        while (endCheck) {
            if (links[index].parentIndex == -1)
                endCheck = false;
            else {
                ret.push_back(links[index].parentIndex);
                index = links[index].parentIndex;
            }
        }
        return ret;
    }
    const std::vector<int> getAncestors( int start, int level, int end = 0 ) {
        std::vector<int> ret;
        int index = start;
        bool endCheck = 1;
        
        while(endCheck){
            if(links[index].parentIndex == -1)
                endCheck = false;
            else{
                if (level == links[links[index].parentIndex].level)
                    ret.push_back(links[index].parentIndex);
                if (links[links[index].parentIndex].isLevelRoot) endCheck = false;
                if (links[index].parentIndex == end) endCheck = false;
                index = links[index].parentIndex;
            }
        }
        return ret;
    }

    const vector<int> GetChidren(int start) {
        vector<int> ret;
        int index = start;
        bool endCheck = 1;
        int childIndex = -1;
        
        while (endCheck) {
            childIndex = links[index].childIndex;
            if (childIndex == -1 || links[childIndex].isHand || links[childIndex].isEnd)
                endCheck = false;
            else {
                ret.push_back(childIndex);
                index = childIndex;
            }
        }
        return ret;
    }
    void SetSkeletonLevel(const int level) {
        if (level == 1) {
            for (auto& index : LEVEL_1) {
                links[index].level = level;
            }
        }
        else if (level == 2) {
            for (auto& index : LEVEL_2) {
                links[index].level = level;
            }
        }
    }
    void updateLink(){
        for (int i = 1; i < links.size(); i++) {
            Link pLink = links[links[i].parentIndex];
            links[i].updatePose(pLink.getPos(), pLink.getOri());
        }
    }
};

struct Motion {
    std::vector<Body> bodies;
    unsigned int motionNum;
    unsigned int frameNum;
    
    void clear() {
        bodies.clear();
        vector<Body>().swap(bodies);
    }
    void add(const Body &body) {
        bodies.push_back(body);
    }
    void SimplifySkeleton(const int level) {
        for (auto& body : bodies) {
            body.SetSkeletonLevel(level);
            for (auto& link : body.links) {
                if (level > link.level) {
                    link.q = quat(1, 0, 0, 0);
                    link.isFixed = true;
                }
            }
            body.updateLink();
        }
    }
};


#endif /* motion_hpp */
