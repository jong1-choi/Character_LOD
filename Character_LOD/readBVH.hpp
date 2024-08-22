#ifndef readBVH_hpp
#define readBVH_hpp
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <glm/gtx/quaternion.hpp>
#include "motion.hpp"

using namespace  std;

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
        string               name;
        unsigned int         index;
        Joint *              parent;
        vector< Joint * >    children;
        float                offset[3];
        bool                 has_site;
        float                site[3];
        vector< Channel * >  channels;
        glm::quat            quat;
        glm::vec3            position;
        float                yRot;
    };

    bool                     is_load_success;
    string                   file_name;
    string                   motion_name;
    unsigned int             num_channel;
    vector< Channel * >      channels;
    vector< Joint * >        joints;
    map< string, Joint * >   joint_index;
    unsigned int             num_frame;
    float                    interval;
    float *                  motion;

    BVH();
    BVH( const char * bvh_file_name );
    ~BVH();

    void Clear();
    
    void Load( const char * bvh_file_name );
    
    bool IsLoadSuccess() const { return is_load_success; }

    const string& GetFileName() const { return file_name; }
    
    const string& GetMotionName() const { return motion_name; }

    const unsigned int GetNumJoint() const { return  joints.size(); }
    
    const Joint* GetJoint( int no ) const { return  joints[no]; }
    
    const unsigned int GetNumChannel() const { return  channels.size(); }
    
    const Channel* GetChannel( int no ) const { return  channels[no]; }

    const Joint* GetJoint( const string & j ) const  {
        map< string, Joint * >::const_iterator  i = joint_index.find( j );
        return  ( i != joint_index.end() ) ? (*i).second : NULL; }
    
    const Joint *   GetJoint( const char * j ) const  {
        map< string, Joint * >::const_iterator  i = joint_index.find( j );
        return  ( i != joint_index.end() ) ? (*i).second : NULL; }

    int GetNumFrame() const { return  num_frame; }
    
    double GetInterval() const { return  interval; }
    
    double GetMotion( int f, int c ) const { return  motion[ f*num_channel + c ]; }

    void SetMotion( int f, int c, float v ) { motion[ f*num_channel + c ] = v; }
    
    void UpdatePose( int frame_no, Body &body, float scale = 1.0f );

    static void UpdatePose( Joint * root, const float* data, Body &body, float scale = 1.0f );

};

#endif // _BVH_H_
