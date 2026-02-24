#ifndef Motion_h
#define Motion_h

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "GLTools.h"

struct Link {
    glm::vec3 parentWorldPos = glm::vec3(0);
    glm::quat parentWorldRot = glm::quat(1, 0, 0, 0);
    glm::vec3 boneOffset     = glm::vec3(0);
    glm::quat localRot       = glm::quat(1, 0, 0, 0);

    bool         isFixed      = false;
    bool         isHand       = false;
    bool         isEnd        = false;
    bool         isLevelRoot  = false;
    unsigned int level        = 0;
    int          index        = -1;
    int          parentIndex  = -1;
    int          childIndex   = -1;

    Link(int parI, int chiN, const glm::vec3& offset, const glm::quat& rot, const glm::quat& parentRot, bool end, bool h, int i);

    void shapeRender(glm::vec3 offset, glm::vec4 color = glm::vec4(1, 1, 1, 0), float size = 1, float multiple = 2) const;

    glm::vec3 getWorldPos() const;
    glm::vec3 getWorldPos(float multiplier) const;
    glm::vec3 getBoneVector() const;
    glm::vec3 getNearP(glm::vec3 dir, float scale = 1) const;
    glm::quat getWorldRot() const;

    void applyLocalRotation(const glm::quat& rot);
    void updateParentTransform(const glm::vec3& pos, const glm::quat& ori);
    void updateLevelRoot();
};


struct Body {
    std::vector<Link> links;

    ~Body();
    void clear();
    void add(int parI, int chiI, const glm::vec3& offset, const glm::quat& rot, const glm::quat& parentRot, bool end, bool h, int i);
    void shapeRender(glm::vec3 offset = glm::vec3(0), glm::vec4 color = glm::vec4(1, 1, 1, 0), float size = 0.4f) const;

    std::vector<int> getAncestors(int end);
    std::vector<int> getAncestors(int start, unsigned int level, int end = 0);
    std::vector<int> getChildren(int start);

    void setSkeletonLevel(unsigned int level);
    void updateLink();
};

struct Motion {
    std::vector<Body> bodies;
    unsigned int totalFrames = 0;

    void clear();
    void add(const Body& body);
    void simplifySkeleton(unsigned int level);
};


#endif /* Motion_h */
