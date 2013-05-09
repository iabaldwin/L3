#ifndef L3_CONTROLLERS_H
#define L3_CONTROLLERS_H

#include <GLV/glv.h>
//#ifdef __APPLE__
//#include <GLUT/glut.h>
//#else
//#include <GL/glut.h>
//#endif

#include <iostream>

#include "L3.h"

namespace L3
{
namespace Visualisers
{

struct control_t
{
    control_t();
    
    control_t( float x, float y, float z, float r, float p, float q );

    control_t& operator +=( control_t& rhs  );

    control_t& translateZ( float z );

    control_t& updateHomogeneous();

    Eigen::Matrix4f homogeneous;

    double x,y,z,r,p,q;
};

void convert( const control_t& control, Eigen::Matrix4f& mat );

/*
 *  Core controller class
 */
struct Controller 
{

    Controller( control_t& control ) : current(control)
    {
    }

    virtual void onEvent( glv::Event::t type, glv::GLV& g ) = 0;

    glv::space_t origin_x, origin_y;
    
    control_t& current;

};

/*
 *  Basic panning motion
 */
struct BasicPanController : Controller
{
    BasicPanController( control_t& control ) : Controller(control)
    {
    }

    void onEvent( glv::Event::t type, glv::GLV& g );
    

};

/*
 *  FPS Motion
 */
struct FPSController : Controller
{
    FPSController( control_t& control ) : Controller(control)
    {
    }

    void onEvent( glv::Event::t type, glv::GLV& g );
};


}
}


#endif
