#pragma once

#include "RenderCore.h"
#include "Components.h"

#include <set>

namespace L3
{
  namespace Visualisers
  {

    struct Action
    {
      virtual void apply( glv::View* v ) = 0;
      virtual void invert( glv::View* v ){}; 

    };

    struct NoAction : Action
    {
      void apply( glv::View* v )
      {
      }
    };

    struct SelectAction : Action
    {
      // Apply the action
      void apply( glv::View* view )
      {
      }

      void invert(glv::View* view )
      {
      }
    };

    struct HighLightAction : Action
    {
      void apply( glv::View* v )
      {
      }
    };

    struct Maximise : Action
    {
      virtual void apply( glv::View* v )
      {
        v->maximize();
        v->bringToFront();
      }
    };

    struct Toggle: Action
    {
      virtual void apply( glv::View* v )
      {
        if ( v->enabled( glv::Property::Maximized ) )
          v->restore();
        else
          v->maximize();
      }
    };

    template <typename T>
      struct Increment : Action
    {

      Increment( T& t ) : t(t)
      {

      }

      T& t ;

      void apply( glv::View* )
      {
        t++;
      }
    };

    struct EventController : glv::EventHandler
    {
      EventController( glv::View* view, glv::Event::t type, Action* action  ) 
        :  view(view), 
        action(action),
        last_down(0.0)
      {
        view->addHandler( type, *this );
      }

      glv::View*  view;
      Action*     action;

      double last_down;
      L3::Timing::ChronoTimer t;

      virtual bool onEvent( glv::View& v, glv::GLV& g)
      {
        if (( t.elapsed() - last_down ) < .2 )
        {
          action->apply( view ); 
          // Debouncer 
          last_down = 0.0; 
        }
        else
          last_down  = t.elapsed();

        return false;
      }
    };

    struct DoubleClickController : EventController
    {

      DoubleClickController( glv::View* view, Action* action ) 
        : EventController( view, glv::Event::MouseDown, action )
      {
      }

    };

    struct DoubleClickMaximiseToggle : DoubleClickController
    {
      DoubleClickMaximiseToggle( glv::View* view ) : DoubleClickController( view, new Toggle() )
      {
      }
    };

    template <typename T>
      struct Incrementer : EventController
    {
      Incrementer( glv::View* view, T t  ) 
        : EventController( view,  static_cast< glv::Event::t >( DBG_X ), new Increment<T>(t))
      {
      }
    };

    struct MouseQuery : EventController
    {
      MouseQuery( glv::View3D* view, Action* action  ) 
        : EventController( view, glv::Event::MouseDown, action ) ,
        composite( dynamic_cast< L3::Visualisers::Composite* >(view))
      {
      }

      L3::Visualisers::Composite* composite;

      virtual bool onEvent( glv::View& v, glv::GLV& g)
      {
        return false; //Don't bubble
      }

    };

    struct MouseQuerySelect : MouseQuery
    {
      MouseQuerySelect( glv::View3D* view ) 
        : MouseQuery( view, new SelectAction() )
      {
      }
    };

    struct InputManager : glv::EventHandler
    {
    };

    struct WASDController : InputManager
    {
      bool onEvent( glv::View& v, glv::GLV& g )
      {
        return false;
      }
    };

    struct SelectionManager
    {
      SelectionManager( MouseQuerySelect* SELECT, InputManager* INPUT ) : select(SELECT), input(INPUT)
      {
      }

      InputManager*       input;
      MouseQuerySelect*   select;
    };

    struct WASDManager : SelectionManager
    {
      WASDManager( MouseQuerySelect* selector) : SelectionManager( selector, new WASDController() )
      {
      }

    };

    struct CompositeLeafViewToggle : glv::Buttons
    {

      CompositeLeafViewToggle( boost::shared_ptr< L3::Visualisers::Leaf > leaf, std::string name, const glv::Rect& r= glv::Rect(), int nx=1, int ny=1,bool momentary=false, bool mutExc=false )
        : glv::Buttons( r, nx, ny, momentary, mutExc ), leaf(leaf)
      {

        label = boost::make_shared< glv::Label >( name );
        label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 

        *this << *label;
      }

      boost::shared_ptr< glv::Label > label;

      boost::weak_ptr< L3::Visualisers::Leaf > leaf;

      bool toggle_val;

      bool onEvent( glv::Event::t e, glv::GLV& g )
      {
        glv::Buttons::onEvent(e,g);

        boost::shared_ptr< L3::Visualisers::Leaf > leaf_ptr = leaf.lock();

        if ( leaf_ptr )
          leaf_ptr->visible = getValue();

        return false;
      }
    };
  } // Visualisers
} // L3
