#pragma once

#include <iostream>
#include <glv.h>
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

    /*
     *  Core controller class
     */
    struct Controller 
    {
      Controller( control_t& position ) : current(position)
      {
      }

      virtual ~Controller(){}

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
    };

    /*
     *  FPS Motion
     */
    struct FPSController : Controller
    {
      FPSController( control_t& control ) : Controller(control)
      {
      }

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

          return false;
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

        bool onEvent(glv::View&, glv::GLV& g);
      };

      boost::shared_ptr< MouseDownController >    mouse_down_controller;
      boost::shared_ptr< MouseDragController >    mouse_drag_controller;

      CompositeController( glv::View3D* view, control_t& position ) : Controller(position)
      {
        mouse_down_controller = boost::make_shared< MouseDownController >( boost::ref( origin_x), boost::ref( origin_y ));
        mouse_drag_controller = boost::make_shared< MouseDragController >( boost::ref( current), boost::ref( origin_x ), boost::ref( origin_y ) );

        view->addHandler( glv::Event::MouseDown, *mouse_down_controller );
        view->addHandler( glv::Event::MouseDrag, *mouse_drag_controller );
      }
    };

  } // Visualisers
} // L3
