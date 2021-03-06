#pragma once

#include <deque>
#include <iostream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include <boost/shared_ptr.hpp>

namespace L3
{
  namespace Visualisers
  {

    enum CUSTOM_EVENT_TYPES
    {
      OVERLAY_TOGGLE = 20,
      SELECT_CLICK,
      TABLE_TOGGLE,
      DBG_X,
      DBG_Y
    };

    /*
     *  Custom GLV view
     */
    struct L3GLV : glv::GLV
    {

      L3GLV() : broadcast_enabled(true)
      {

      }

      bool broadcast_enabled;

      bool onEvent( glv::Event::t e, glv::GLV& g)
      {
        glv::space_t a =0.0f;
        glv::space_t b =0.0f;

        if ( e == glv::Event::KeyDown )
        {
          const glv::Keyboard& k = g.keyboard();

          switch (k.key())
          {
            case '`':
              this->broadcastEvent( static_cast< glv::Event::t>( OVERLAY_TOGGLE ) );

              // This is quite grim
              this->setMouseDown(a, b, 1, 1);
              this->setMouseUp(a, b, 1, 1);
              break;

            case glv::Key::Tab:
              if( broadcast_enabled )
                this->broadcastEvent( static_cast< glv::Event::t>( TABLE_TOGGLE ) );
              break;

            case '+':
              if( broadcast_enabled )
                this->broadcastEvent( static_cast< glv::Event::t>( DBG_X ) );
              break;

            default:
              break;
          }
        }

        return false;
      }
    };

    struct  CustomTable : glv::Table
    {
      CustomTable( const char * arrangement="<", glv::space_t padX=3, glv::space_t padY=3, const glv::Rect& r= glv::Rect(0))
        : Table( arrangement, padX, padY, r )
      {
        this->enable( glv::DrawBack | glv::Controllable);
      }

      bool onEvent( glv::Event::t type, glv::GLV& g );
    };

    struct TableToggler : glv::View
    {
      TableToggler( const glv::Rect& rect, std::deque< boost::shared_ptr< glv::Table > > * tables );

      std::deque< boost::shared_ptr< glv::Table > > * tables;
      int current_table, num_tables;
      boost::shared_ptr< glv::Buttons > page_pointer;

      bool onEvent( glv::Event::t type, glv::GLV& g );
    };

  } // Visualisers
} // L3
