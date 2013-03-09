#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>

#include <GLV/glv.h>

#include "L3.h"


namespace L3
{

namespace Visualisers
{

struct CoordinateSystem
{

    CoordinateSystem()  : _pose( L3::SE3::ZERO() )
    {
        _init();
    }

    CoordinateSystem( const L3::SE3& pose ) : _pose( pose )
    {
        _init();
    }
 
    L3::SE3 _pose;

    ~CoordinateSystem()
    {
        delete [] vertices;
    }

    void _init()
    {
        vertices = new glv::Point3[6];
        vertices[0]( 0, 0, 0 ); 
        vertices[1]( 1, 0, 0 ); 
        vertices[2]( 0, 0, 0 ); 
        vertices[3]( 0, 1, 0 ); 
        vertices[4]( 0, 0, 0 ); 
        vertices[5]( 0, 0, 1 ); 
   
        colors = new glv::Color[6];
        colors[0] = glv::Color( 1, 0, 0 ); 
        colors[1] = glv::Color( 1, 0, 0 ); 
        colors[2] = glv::Color( 0, 1, 0 ); 
        colors[3] = glv::Color( 0, 1, 0 ); 
        colors[4] = glv::Color( 0, 0, 1 ); 
        colors[5] = glv::Color( 0, 0, 1 ); 
    }

    void onDraw3D(glv::GLV& g)
    { 
        glv::draw::push();
        glv::draw::translate( _pose.x, _pose.y, _pose.z );
        glv::draw::paint( glv::draw::LineStrip, vertices, colors, 6 );
        glv::draw::pop();
    }

    glv::Color* colors;
    glv::Point3* vertices;

};


}
}



#endif
