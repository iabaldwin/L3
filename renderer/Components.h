#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>
#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>


namespace L3
{

struct Component : glv::View3D{

    Component()
    {
        stretch(1,1); 
    }
	
	virtual void onDraw3D(glv::GLV& g)
    {
    
    }

};

struct PoseChain : Component
{

    PoseChain()
    {

    }

    glv::Point3 vertices[20];
    glv::Color colors[20];

    virtual void onDraw3D(glv::GLV& g)
    {
        for ( unsigned int i=0; i<20; i++ )
        {
            float x = random() % 10;
            float y = random() % 10;
            float z = random() % 10;

            vertices[i]( x, y, z );

             colors[i] = glv::HSV(0.6, .1, z*0.45+0.55);
       
             glv::draw::translateZ( -4 );
        }
   
        glv::draw::paint( glv::draw::Points, vertices, colors, 20 );
   
    }

};

}
#endif
