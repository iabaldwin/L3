#include "Controller.h"

namespace L3
{
namespace Visualisers
{
    control_t::control_t() : x(0), y(0), z(0), r(0), p(0), q(0)
    {
        updateHomogeneous();
    }
    
    control_t::control_t( float x, float y, float z, float r, float p, float q ) : x(x), y(y), z(z), r(r), p(p), q(q)
    {
        updateHomogeneous();
    }

    control_t& control_t::operator+=( control_t& rhs  )
    {
        //this->homogeneous *= rhs.homogeneous;
        this->homogeneous = (rhs.homogeneous *= this->homogeneous );

        //this->x += rhs.x;
        //this->y += rhs.y;
        //this->z += rhs.z;
        //this->r += rhs.r;
        //this->p += rhs.p;
        //this->q += rhs.q;

        return *this;
    }

    control_t& control_t::translateZ( float z )
    {
        this->z = z;
        this->updateHomogeneous();
        return *this;
    }

    control_t& control_t::updateHomogeneous()
    {
        Eigen::Matrix4f Rz = Eigen::Matrix4f::Identity();
        Rz(0,0) = cos( q );
        Rz(0,1) = -1*sin( q );
        Rz(1,0) = sin( q );
        Rz(1,1) = cos( q );

        Eigen::Matrix4f Rx = Eigen::Matrix4f::Identity();
        Rx(1,1) = cos( p );
        Rx(1,2) = -1*sin( p );
        Rx(2,1) = sin( p );
        Rx(2,2) = cos( p );

        Eigen::Matrix4f Ry = Eigen::Matrix4f::Identity();
        Ry(0,0) = cos( r );
        Ry(0,2) = sin( r );
        Ry(2,0) = -1*sin( r );
        Ry(2,2) = cos( r );

        homogeneous = Rz*Ry*Rx;

        // SE3
        homogeneous(0,3) = x;
        homogeneous(1,3) = y;
        homogeneous(2,3) = z;
   
        return *this;
    }



/*
 *  Basic pan controller
 */
void BasicPanController::onEvent( glv::Event::t type, glv::GLV& g )
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
             
                if ( g.keyboard().shift() )
                {
                    t.x = (double)(g.mouse().x() - origin_x) /100;
                    t.y = -1*(double)(g.mouse().y() - origin_y) /100;
                }

                else
                {
                    t.q = -1*(double)(g.mouse().x() - origin_x) /10000;
                    t.p = -1*(double)(g.mouse().y() - origin_y) /10000;
                    
                    // Clamp
                    //t.r = pitch;
                    //pitch = (double)(g.mouse().y() - origin_y) /1000;

                    //if (estimate.r > -60 || (pitch > 0 ))
                        //t.r = pitch;
               
                }
                                
                break;

            case (glv::Event::MouseUp):
                break;

            default:
                break;

        }

        current += (t.updateHomogeneous());
        
    }

/*
 *  FPS controller
 */

void FPSController::onEvent( glv::Event::t type, glv::GLV& g )
{
        Eigen::Matrix4f homogeneous;

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
        //return control_t();
}

}
}
