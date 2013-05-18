#ifndef L3_VIEW_CONTROLLERS_H
#define L3_VIEW_CONTROLLERS_H

#include <iostream>
#include <GLV/glv.h>
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
    control_t& updateEuler();

    Eigen::Matrix4f homogeneous;

    double x,y,z,r,p,q;
};

void convert( const control_t& control, Eigen::Matrix4f& mat );

/*
 *  Core controller class
 */
//struct Controller : glv::EventHandler
struct Controller 
{
    Controller( control_t& position ) : current(position)
    {
    }

    //virtual bool onEvent( glv::Event::t type, glv::GLV& g ) = 0;
    //virtual bool onEvent(glv::View&, glv::GLV&) = 0;

    //glv::space_t origin_x, origin_y;
    
    control_t& current;

};

struct SceneController : Controller
{

    SceneController( control_t& position ) : Controller(position)
    {
    }

    bool onEvent( glv::Event::t type, glv::GLV& g );

};


/*
 *  Basic panning motion
 */
struct BasicPanController : Controller
{
    BasicPanController( control_t& position ) : Controller(position)
    {
    }

    bool onEvent( glv::Event::t type, glv::GLV& g );
    //bool onEvent(glv::View&, glv::GLV&);
    

};

/*
 *  FPS Motion
 */
struct FPSController : Controller
{
    FPSController( control_t& control ) : Controller(control)
    {
    }

    //bool onEvent( glv::Event::t type, glv::GLV& g );
    bool onEvent(glv::View&, glv::GLV&);
};


struct CompositeController : Controller
{
    
    double origin_x, origin_y;

    struct MouseDownController : glv::EventHandler
    {

        MouseDownController( double& origin_x, double& origin_y ) 
            : origin_x(origin_x), origin_y(origin_y)
        {

        }

        double& origin_x;
        double& origin_y;

        bool onEvent(glv::View&, glv::GLV& g)
        {
            origin_x = g.mouse().x();
            origin_y = g.mouse().y();
        }

    };

    struct MouseDragController : glv::EventHandler
    {
        MouseDragController( control_t& current, double& origin_x, double& origin_y ) 
            : current(current), origin_x(origin_x), origin_y(origin_y)
        {

        }
        control_t& current;

        double& origin_x;
        double& origin_y;

        bool onEvent(glv::View&, glv::GLV& g)
        {
            current.q += (double)(g.mouse().x() - origin_x) /100;

            double r = (double)(g.mouse().y() - origin_y) /100;

            if ( ( current.r > -65 && r < 0 ) || ( current.r < 65 && r > 0 ) )
                current.r += r;
        }

    };

    boost::shared_ptr< MouseDownController >    mouse_down_controller;
    boost::shared_ptr< MouseDragController >    mouse_drag_controller;

    CompositeController( glv::View3D* view, control_t& position ) : Controller(position)
    {
        mouse_down_controller.reset( new MouseDownController( origin_x, origin_y ) );
        mouse_drag_controller.reset( new MouseDragController( current, origin_x, origin_y ) );

        view->addHandler( glv::Event::MouseDown, *mouse_down_controller );
        view->addHandler( glv::Event::MouseDrag, *mouse_drag_controller );
    }

};



}
}


#endif
