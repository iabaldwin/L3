#ifndef L3_CONTROLLERS_H
#define L3_CONTROLLERS_H

#include <GLV/glv.h>
#include <GLUT/glut.h>

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

struct Controller 
{
    glv::space_t origin_x, origin_y;

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
                origin_x = g.mouse().x();
                origin_y = g.mouse().y();
                break;

            case (glv::Event::MouseDrag):
                t.q =  (double)(g.mouse().x() - origin_x) /100;
                t.r =  (double)(g.mouse().y() - origin_y) /100;
                break;

            case (glv::Event::MouseUp):
                break;

            default:
                break;

        }

        return t;
    }

};


}
}


#endif
