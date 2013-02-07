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
        std::cout << "FUCK" << std::endl;
    }

};

typedef std::vector<L3::Pose*>::iterator POSE_CHAIN_ITERATOR;

struct PoseChain : Component
{

    glv::Color colors[20];
    glv::Point3 vertices[20];

    PoseChain( std::vector<L3::Pose*>& POSES ) : poses(POSES)
    {
    }

    std::vector<L3::Pose*>& poses;
    
    virtual void onDraw3D(glv::GLV& g)
    {
        int counter = 0;
        for( POSE_CHAIN_ITERATOR it=poses.begin(); it < poses.end(); it++ )
        {
            std::cout << *(*it) << std::endl;
            if ( counter++ == 20 )
                break;
        }

        //for ( unsigned int i=0; i<20; i++ )
        //{
            //float x = random() % 10;
            //float y = random() % 10;
            //float z = random() % 10;

            //vertices[i]( x, y, z );

             //colors[i] = glv::HSV(0.6, .1, z*0.45+0.55);
       
             //glv::draw::translateZ( -4 );
        //}
   
        //glv::draw::paint( glv::draw::Points, vertices, colors, 20 );
   
    }

    
};

}
}
#endif
