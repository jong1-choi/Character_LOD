#define _CRT_SECURE_NO_WARNINGS

#include <fstream>
#include <string.h>
#include <glm/gtx/quaternion.hpp>
#include <glm/glm.hpp>
#include "GLTools.hpp"
#include "readBVH.hpp"
#include "motion.hpp"

#define PI 3.141592f


using namespace std;
using namespace glm;

BVH::BVH()
{
    motion = NULL;
    Clear();
}

BVH::BVH( const char * bvh_file_name )
{
    motion = NULL;
    Clear();

    Load( bvh_file_name );
}

BVH::~BVH()
{
    Clear();
}

void  BVH::Clear()
{
    int  i;
    for ( i=0; i<channels.size(); i++ )
        delete  channels[ i ];
    for ( i=0; i<joints.size(); i++ )
        delete  joints[ i ];
    
    if ( motion != NULL )
        delete  motion;

    is_load_success = false;
    
    file_name = "";
    motion_name = "";
    
    num_channel = 0;
    channels.clear();
    vector<Channel*>().swap(channels);
    joints.clear();
    vector<Joint*>().swap(joints);
    joint_index.clear();
    map<string, Joint *>().swap(joint_index);
    
    num_frame = 0;
    interval = 0.0;
    motion = NULL;
}

void  BVH::Load( const char * bvh_file_name )
{
    #define  BUFFER_LENGTH  1024*32

    ifstream  file;
    char      line[ BUFFER_LENGTH ];
    char *    token;
    char      separater[] = " :,\t\r";
    vector< Joint * >   joint_stack;
    Joint *   joint = NULL;
    Joint *   new_joint = NULL;
    bool      is_site = false;
    double    x, y ,z;
    int       i, j;

    Clear();

    file_name = bvh_file_name;
    const char *  mn_first = bvh_file_name;
    const char *  mn_last = bvh_file_name + strlen( bvh_file_name );
    if ( strrchr( bvh_file_name, '\\' ) != NULL )
        mn_first = strrchr( bvh_file_name, '\\' ) + 1;
    else if ( strrchr( bvh_file_name, '/' ) != NULL )
        mn_first = strrchr( bvh_file_name, '/' ) + 1;
    if ( strrchr( bvh_file_name, '.' ) != NULL )
        mn_last = strrchr( bvh_file_name, '.' );
    if ( mn_last < mn_first )
        mn_last = bvh_file_name + strlen( bvh_file_name );
    motion_name.assign( mn_first, mn_last );

    file.open( bvh_file_name, ios::in );
    if ( file.is_open() == 0 )  return;

    while ( ! file.eof() )
    {
        if ( file.eof() )
            goto bvh_error;

        file.getline( line, BUFFER_LENGTH );
        token = strtok( line, separater );

        if ( token == NULL )  continue;

        if ( strcmp( token, "{" ) == 0 )
        {
            joint_stack.push_back( joint );
            joint = new_joint;
            continue;
        }
        if ( strcmp( token, "}" ) == 0 )
        {
            joint = joint_stack.back();
            joint_stack.pop_back();
            is_site = false;
            continue;
        }

        if ( ( strcmp( token, "ROOT" ) == 0 ) ||
             ( strcmp( token, "JOINT" ) == 0 ))
        {
            new_joint = new Joint();
            new_joint->index = joints.size();
            new_joint->parent = joint;
            new_joint->has_site = false;
            new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
            new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;
            joints.push_back( new_joint );
            if ( joint )
                joint->children.push_back( new_joint );

            token = strtok( NULL, "" );
            while ( *token == ' ' )  token ++;
            new_joint->name = token;

            joint_index[ new_joint->name ] = new_joint;
            continue;
        }
        if ( ( strcmp( token, "End" ) == 0 ))
        {
            new_joint = new Joint();
            new_joint->index = joints.size();
            new_joint->parent = joint;
            new_joint->has_site = true;
            new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
            new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;
            joints.push_back( new_joint );
            if ( joint )
                joint->children.push_back( new_joint );

            token = strtok( NULL, "" );
            while ( *token == ' ' )  token ++;
            new_joint->name = token;

            joint_index[ new_joint->name ] = new_joint;
            continue;
        }
        if ( strcmp( token, "OFFSET" ) == 0 )
        {
            token = strtok( NULL, separater );
            x = token ? atof( token ) : 0.0;
            token = strtok( NULL, separater );
            y = token ? atof( token ) : 0.0;
            token = strtok( NULL, separater );
            z = token ? atof( token ) : 0.0;

            joint->offset[0] = float(x);
            joint->offset[1] = float(y);
            joint->offset[2] = float(z);
            continue;
        }

        if ( strcmp( token, "CHANNELS" ) == 0 )
        {
            token = strtok( NULL, separater );
            joint->channels.resize( token ? atoi( token ) : 0 );

            for ( i=0; i<joint->channels.size(); i++ )
            {
                Channel *  channel = new Channel();
                channel->joint = joint;
                channel->index = channels.size();
                
                channels.push_back( channel );
                joint->channels[ i ] = channel;

                token = strtok( NULL, separater );
                if ( strcmp( token, "Xrotation" ) == 0 )
                    channel->type = X_ROTATION;
                else if ( strcmp( token, "Yrotation" ) == 0 )
                    channel->type = Y_ROTATION;
                else if ( strcmp( token, "Zrotation" ) == 0 )
                    channel->type = Z_ROTATION;
                else if ( strcmp( token, "Xposition" ) == 0 )
                    channel->type = X_POSITION;
                else if ( strcmp( token, "Yposition" ) == 0 )
                    channel->type = Y_POSITION;
                else if ( strcmp( token, "Zposition" ) == 0 )
                    channel->type = Z_POSITION;
            }
        }

        if ( strcmp( token, "MOTION" ) == 0 )
            break;
    }

    file.getline( line, BUFFER_LENGTH );
    token = strtok( line, separater );
    if ( strcmp( token, "Frames" ) != 0 )
        goto bvh_error;
    token = strtok( NULL, separater );
    if ( token == NULL )
        goto bvh_error;
    num_frame = atoi( token );

    file.getline( line, BUFFER_LENGTH );
    token = strtok( line, ":" );
    if ( strcmp( token, "Frame Time" ) != 0 )
        goto bvh_error;
    token = strtok( NULL, separater );
    if ( token == NULL )
        goto bvh_error;
    interval = atof( token );

    num_channel = channels.size();
    motion = new float[ num_frame * num_channel ];

    for ( i=0; i<num_frame; i++ )
    {
        file.getline( line, BUFFER_LENGTH );
        token = strtok( line, separater );
        for ( j=0; j<num_channel; j++ )
        {
            if ( token == NULL )
                goto bvh_error;
            motion[ i*num_channel + j ] = atof( token );
            token = strtok( NULL, separater );
        }
    }

    file.close();

    is_load_success = true;
    if(is_load_success) cout << "load complete" << endl;

    return;

bvh_error:
    file.close();
}

void  BVH::UpdatePose( int frame_num, Body &body, float scale )
{
    UpdatePose( joints[0], motion + frame_num * num_channel, body, scale );
}


void  BVH::UpdatePose( Joint * joint, const float* data, Body &body, float scale)
{
    Joint *parent = joint->parent;
    bool isHand = false;
    
    joint->quat = glm::quat(1,0,0,0);
    
    for ( int i=0; i < joint->channels.size(); i++ )
    {
        Channel *  channel = joint->channels[ i ];
        
        if (data[channel->index] == 0) continue;

        if ( channel->type == X_ROTATION )
            joint->quat *= glm::exp(data[channel->index]*PI/180/2 * quat(0,1,0,0));
        else if ( channel->type == Y_ROTATION ){
            joint->quat *= glm::exp(data[channel->index]*PI/180/2 * quat(0,0,1,0));
        }
        else if ( channel->type == Z_ROTATION )
            joint->quat *= glm::exp(data[channel->index]*PI/180/2 * quat(0,0,0,1));

        joint->quat = normalize(joint->quat);
    }
    
    if( joint->name.find("Thumb") != string::npos) isHand = true;
    else if( joint->name.find("Index") != string::npos) isHand = true;
    else if( joint->name.find("Middle") != string::npos) isHand = true;
    else if( joint->name.find("Ring") != string::npos) isHand = true;
    else if( joint->name.find("Pinky") != string::npos) isHand = true;
    
    if ( parent == NULL )
    {
        joint->position = scale * glm::vec3(data[ 0 ], data[ 1 ], data[ 2 ] );
        //body.add(-1, joint->children[0]->index, joint->position, joint->quat);
        body.add(-1, joint->children[0]->index, joint->position, joint->quat, quat(1,0,0,0), joint->has_site, isHand, joint->index);
    }
    else if( joint->has_site ){
        joint->quat = quat(1,0,0,0);
        glm::vec3 offset = scale * glm::vec3(joint->offset[ 0 ], joint->offset[ 1 ], joint->offset[ 2 ]);
        joint->position = parent->quat * offset + parent->position;
        //body.add(parent->index, -1, offset, joint->quat);
        body.add(parent->index, -1, offset, joint->quat, parent->quat, joint->has_site, isHand, joint->index);
    }
    else
    {
        glm::vec3 offset = scale * glm::vec3(joint->offset[ 0 ], joint->offset[ 1 ], joint->offset[ 2 ]);
        joint->position = parent->quat * offset + parent->position;
        
        //body.add(parent->index, joint->children[0]->index, offset, joint->quat);
        body.add(parent->index, joint->children[0]->index, offset, joint->quat, parent->quat, joint->has_site, isHand, joint->index);
    }
    for ( int i=0; i<joint->children.size(); i++ )
    {
        UpdatePose( joint->children[ i ], data, body, scale );
    }

}
