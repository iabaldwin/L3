#ifndef L3_CONTROLLERS_H
#define L3_CONTROLLERS_H

#include <GLV/glv.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <iostream>

#include "L3.h"

namespace L3
{
namespace Visualisers
{

struct control_t
{
    control_t() : x(0), y(0), z(0), r(0), p(0), q(0)
    {}
    
    control_t( float x, float y, float z, float r, float p, float q ) : x(x), y(y), z(z), r(r), p(p), q(q)
    {}
    

    control_t& operator +=( control_t& rhs  )
    {
        this->x += rhs.x;
        this->y += rhs.y;
        this->z += rhs.z;
        
        this->r += rhs.r;
        this->p += rhs.p;
        this->q += rhs.q;

        return *this;
    }

    double x,y,z,r,p,q;
};


control_t operator+( const control_t& a, const control_t& b );
std::ostream& operator<<( std::ostream& o, const control_t& t );

void convert( const control_t& control, Eigen::Matrix4f& mat );

/*
 *Core controller class
 */
struct Controller 
{

    Controller()
    {
        //glutIgnoreKeyRepeat(0);
    }

    virtual control_t onEvent( glv::Event::t type, glv::GLV& g ) = 0;

    control_t estimate;
    
};

/*
 *Basic panning motion
 */
struct BasicPanController : Controller
{
    glv::space_t origin_x, origin_y;

    control_t onEvent( glv::Event::t type, glv::GLV& g )
    {
        control_t t;
      
        double pitch;

        switch (type)
        {
            case (glv::Event::KeyDown):
                break;

            case (glv::Event::KeyRepeat):
                break;

            case (glv::Event::MouseDown):
                origin_x = g.mouse().x();
                origin_y = g.mouse().y();
                break;

            case (glv::Event::MouseDrag):
             
                t.q =  (double)(g.mouse().x() - origin_x) /100;
               
                // Clamp
                pitch = (double)(g.mouse().y() - origin_y) /100;
                
                if (estimate.r > -60 || (pitch > 0 ))
                    t.r = pitch;
                
                break;

            case (glv::Event::MouseUp):
                break;

            default:
                break;

        }

        estimate = estimate + t;
        return t;
    }

};

/*
 *  FPS Motion
 */
struct FPSController : Controller
{
    glv::space_t origin_x, origin_y;

    control_t onEvent( glv::Event::t type, glv::GLV& g );
};


}
}


#endif
