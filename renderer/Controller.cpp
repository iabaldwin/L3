#include "Controller.h"

namespace L3
{
namespace Visualisers
{


std::ostream& operator<<( std::ostream& o, const control_t& t )
{
    //o << t.x << ":" << t.y << ":" << t.z << ":" << t.r << ":" << t.p << ":" << t.q;
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

void convert( const control_t& control, Eigen::Matrix4f& mat )
{
    mat = L3::SE3( control.x, control.y, control.z, control.r, control.p, control.q ).getHomogeneous();

}

/*
 *FPS controller
 */

control_t FPSController::onEvent( glv::Event::t type, glv::GLV& g )
{
        Eigen::Matrix4f homogeneous;
        
        convert( estimate, homogeneous );

        control_t delta;

        switch (type)
        {
            case (glv::Event::KeyDown):
            case (glv::Event::KeyRepeat):
                //std::cout << g.keyboard().key() << std::endl;
                switch ( g.keyboard().key() )
                {
                    case 'w':
                        delta.y += .5;
                }

                break;

            case (glv::Event::MouseDown):
                break;

            case (glv::Event::MouseDrag):
                break;

            case (glv::Event::MouseUp):
                break;

            case (glv::Event::MouseWheel):
                break;

            default:
                break;

        }

        //estimate = estimate + t;
        //return t;
        
        return control_t();
}

}
}
