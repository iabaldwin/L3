#include "ViewController.h"

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

    control_t& control_t::updateEuler()
    {
        //L3::SE3 pose = L3::Utils::Math::poseFromRotation( this->homogeneous );
   
        //this->x = pose.X();
        //this->y = pose.Y();
        //this->z = pose.Z();
        //this->r = pose.R();
        //this->p = pose.P();
        //this->q = pose.Q();
   
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
                    double x = (double)(g.mouse().x() - origin_x) /100;
                    double y = (double)(g.mouse().y() - origin_y) /100;
               
                    Eigen::Matrix4f delta = Eigen::Matrix4f::Identity(); 
          
                    delta( 0,3)= x;
                    delta( 2,3)= y;

                    current.homogeneous *= delta;
                }
                else
                {
                    current.q += (double)(g.mouse().x() - origin_x) /100;
                   
                    double r = (double)(g.mouse().y() - origin_y) /100;

                    if ( ( current.r > -65 && r < 0 ) || ( current.r < 65 && r > 0 ) )
                        current.r += r;
        
                }
        
                break;

            case (glv::Event::MouseUp):
                break;

            default:
                break;

        }

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
