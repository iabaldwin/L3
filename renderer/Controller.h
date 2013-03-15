#ifndef L3_CONTROLLERS_H
#define L3_CONTROLLERS_H

#include <GLV/glv.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

namespace L3
{
namespace Visualisers
{

struct control_t
{
    control_t() : x(0), y(0), z(0), r(0), p(0), q(0)
    {}
    
    double x,y,z,r,p,q;
};

std::ostream& operator<<( std::ostream& o, const control_t& t )
{
    o << t.x << ":" << t.y << ":" << t.z << ":" << t.r << ":" << t.p << ":" << t.q;
    return o;
}

control_t operator+( const control_t& a, const control_t& b )
{
    control_t retval;

    retval.x = a.x + b.x;
    retval.y = a.y + b.y;
    retval.z = a.z + b.z;
    retval.r = a.r + b.r;
    retval.p = a.p + b.p;
    retval.q = a.q + b.q;

    return retval;
}

/*
 *Core controller class
 */
struct Controller 
{

    virtual control_t onEvent( glv::Event::t type, glv::GLV& g ) = 0;

};

/*
 *Basic panning motion
 */
struct BasicPanController : Controller
{
    glv::space_t origin_x, origin_y;

    control_t estimate;
    
    control_t onEvent( glv::Event::t type, glv::GLV& g )
    {
        control_t t;
      
        double roll;

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
                roll = (double)(g.mouse().y() - origin_y) /100;
                if (estimate.r > -60 || (roll > 0 ))
                    t.r = roll;
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

    control_t estimate;
    
    control_t onEvent( glv::Event::t type, glv::GLV& g )
    {
        control_t t;
      
        switch (type)
        {
            case (glv::Event::KeyDown):
                break;

            case (glv::Event::KeyRepeat):
                break;

            case (glv::Event::MouseDown):
                break;

            case (glv::Event::MouseDrag):
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


}
}


#endif
