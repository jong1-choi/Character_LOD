#ifndef BVH_h
#define BVH_h

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <glm/gtx/quaternion.hpp>
#include "Motion.h"

struct BVH
{
    enum ChannelEnum
    {
        X_ROTATION, Y_ROTATION, Z_ROTATION,
        X_POSITION, Y_POSITION, Z_POSITION
    };
    struct Joint;

    struct Channel
    {
        Joint *              joint;
        ChannelEnum          type;
        unsigned int         index;
    };

    struct Joint
    {
        std::string              name;
        unsigned int             index;
        Joint *                  parent;
        std::vector<Joint *>     children;
        float                    offset[3];
        bool                     has_site;
        float                    site[3];
        std::vector<Channel *>   channels;
        glm::quat                quat;
        glm::vec3                position;
        float                    yRot;
    };

    bool                                 is_load_success;
    std::string                          file_name;
    std::string                          motion_name;
    unsigned int                         num_channel;
    std::vector<Channel *>               channels;
    std::vector<Joint *>                 joints;
    std::map<std::string, Joint *>       joint_index;
    unsigned int                         num_frame;
    float                                interval;
    float *                              motion;

    BVH();
    BVH(const char * bvh_file_name);
    ~BVH();

    void clear();
    void load(const char * bvh_file_name);

    bool isLoadSuccess() const { return is_load_success; }

    const std::string& getFileName() const { return file_name; }

    const std::string& getMotionName() const { return motion_name; }

    unsigned int getNumJoint() const { return (unsigned int)joints.size(); }

    const Joint* getJoint(int no) const { return joints[no]; }

    unsigned int getNumChannel() const { return (unsigned int)channels.size(); }

    const Channel* getChannel(int no) const { return channels[no]; }

    const Joint* getJoint(const std::string& j) const {
        std::map<std::string, Joint *>::const_iterator i = joint_index.find(j);
        return (i != joint_index.end()) ? (*i).second : nullptr; }

    const Joint* getJoint(const char * j) const {
        std::map<std::string, Joint *>::const_iterator i = joint_index.find(j);
        return (i != joint_index.end()) ? (*i).second : nullptr; }

    unsigned int getNumFrame() const { return num_frame; }

    double getInterval() const { return interval; }

    double getMotion(int f, int c) const { return motion[f * num_channel + c]; }

    void setMotion(int f, int c, float v) { motion[f * num_channel + c] = v; }

    void updatePose(int frame_no, Body& body, float scale = 1.0f);

    static void updatePose(Joint * root, const float* data, Body& body, float scale = 1.0f);
};

#endif /* BVH_h */
