#include "Motion.h"

using namespace glm;
using namespace std;

static const int LEVEL_1[10] = { 0, 2, 7, 8, 31, 32, 54, 55, 59, 60 };
static const int LEVEL_2[5]  = { 0, 7, 31, 54, 59 };


// --- Link ---

Link::Link(int parI, int chiN, const glm::vec3& offset, const glm::quat& rot, const glm::quat& parentRot, bool end, bool h, int i)
    : parentIndex(parI), childIndex(chiN), boneOffset(offset), localRot(rot), parentWorldRot(parentRot), isEnd(end), isHand(h), index(i) {
    updateLevelRoot();
}

void Link::updateLevelRoot() {
    if (index == 1 || index == 6 || index == 30 || index == 54 || index == 59) isLevelRoot = true;
}

void Link::shapeRender(glm::vec3 offset, glm::vec4 color, float size, float multiple) const {
    drawCylinder(multiple * (getWorldPos() + offset), multiple * (parentWorldPos + offset), size, color);
    if (!isFixed) drawSphere(multiple * (getWorldPos() + offset), 0.5f, vec4(1, 0, 0, 0));
}

glm::vec3 Link::getWorldPos() const {
    return parentWorldRot * boneOffset + parentWorldPos;
}

glm::vec3 Link::getWorldPos(float multiplier) const {
    if (parentIndex == -1) return boneOffset * localRot;
    return parentWorldRot * boneOffset * multiplier + parentWorldPos;
}

glm::vec3 Link::getBoneVector() const {
    return localRot * boneOffset;
}

glm::vec3 Link::getNearP(glm::vec3 dir, float scale) const {
    return getWorldRot() * dir * scale * 0.2f + getWorldPos();
}

glm::quat Link::getWorldRot() const {
    return parentWorldRot * localRot;
}

void Link::applyLocalRotation(const glm::quat& rot) {
    localRot = rot * localRot;
}

void Link::updateParentTransform(const glm::vec3& pos, const glm::quat& ori) {
    parentWorldPos = pos;
    parentWorldRot = ori;
}


// --- Body ---

Body::~Body() {
    clear();
}

void Body::clear() {
    links.clear();
    vector<Link>().swap(links);
}

void Body::add(int parI, int chiI, const glm::vec3& offset, const glm::quat& rot, const glm::quat& parentRot, bool end, bool h, int i) {
    links.push_back(Link(parI, chiI, offset, rot, parentRot, end, h, i));
}

void Body::shapeRender(glm::vec3 offset, glm::vec4 color, float size) const {
    for (const auto& link : links) {
        if (link.parentIndex >= 0)
            link.shapeRender(offset, color, size);
    }
}

std::vector<int> Body::getAncestors(int end) {
    std::vector<int> ret;
    int index = end;
    bool endCheck = true;
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

std::vector<int> Body::getAncestors(int start, unsigned int level, int end) {
    std::vector<int> ret;
    int index = start;
    bool endCheck = true;
    while (endCheck) {
        if (links[index].parentIndex == -1)
            endCheck = false;
        else {
            if (level == links[links[index].parentIndex].level)
                ret.push_back(links[index].parentIndex);
            if (links[links[index].parentIndex].isLevelRoot) endCheck = false;
            if (links[index].parentIndex == end) endCheck = false;
            index = links[index].parentIndex;
        }
    }
    return ret;
}

std::vector<int> Body::getChildren(int start) {
    std::vector<int> ret;
    int index = start;
    bool endCheck = true;
    int childIdx = -1;
    while (endCheck) {
        childIdx = links[index].childIndex;
        if (childIdx == -1 || links[childIdx].isHand || links[childIdx].isEnd)
            endCheck = false;
        else {
            ret.push_back(childIdx);
            index = childIdx;
        }
    }
    return ret;
}

void Body::setSkeletonLevel(unsigned int level) {
    if (level == 1) {
        for (const auto& idx : LEVEL_1)
            links[idx].level = level;
    }
    else if (level == 2) {
        for (const auto& idx : LEVEL_2)
            links[idx].level = level;
    }
}

void Body::updateLink() {
    for (int i = 1; i < (int)links.size(); i++) {
        const Link& parentLink = links[links[i].parentIndex];
        links[i].updateParentTransform(parentLink.getWorldPos(), parentLink.getWorldRot());
    }
}


// --- Motion ---

void Motion::clear() {
    bodies.clear();
    vector<Body>().swap(bodies);
}

void Motion::add(const Body& body) {
    bodies.push_back(body);
}

void Motion::simplifySkeleton(unsigned int level) {
    for (auto& body : bodies) {
        body.setSkeletonLevel(level);
        for (auto& link : body.links) {
            if (level > link.level) {
                link.localRot = quat(1, 0, 0, 0);
                link.isFixed  = true;
            }
        }
        body.updateLink();
    }
}
