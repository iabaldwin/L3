#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>
#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Datatypes.h"

namespace L3
{
namespace Visual
{

struct Component : glv::View3D{

    Component()
    {
        stretch(1,1); 
    }
	
	virtual void onDraw3D(glv::GLV& g)
    {
    }
	virtual void onDraw2D(glv::GLV& g)
    {
    }


};

typedef std::vector<L3::Pose*>::iterator POSE_CHAIN_ITERATOR;

struct PoseChain : Component
{

    glv::Color* colors;
    glv::Point3* vertices;

    PoseChain( std::vector<L3::Pose*>& POSES ) : poses(POSES)
    {
        colors = new glv::Color[poses.size()];
        vertices = new glv::Point3[poses.size()];

        int counter = 0;
        for( POSE_CHAIN_ITERATOR it=poses.begin(); it < poses.end(); it++ )
        {
            vertices[counter]( (*it)->x, (*it)->y, 0 );
            colors[counter] = glv::HSV(0.6, .1, 0.45+0.55);

            counter++;
        }

        // Far clip
        far( 200 );
    }

    ~PoseChain()
    {
        delete [] colors;
        delete [] vertices;
    }

    std::vector<L3::Pose*>& poses;
   
    void onDraw3D(glv::GLV& g)
    {
        static int k = 0;
        glv::draw::rotateZ( k++ );
        glv::draw::translateZ( -190 );
        glv::draw::paint( glv::draw::Points, vertices, colors, poses.size() );
    }

    void onDraw2D(glv::GLV& g)
    {
    }

};

}
}
#endif
