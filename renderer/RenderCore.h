#ifndef L3_VISUALISERS_RENDER_CORE_H
#define L3_VISUALISERS_RENDER_CORE_H

#include <deque> 
#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include <boost/shared_ptr.hpp>

namespace L3
{
namespace Visualisers
{
 
enum CUSTOM_EVENT_TYPES
    {
        OVERLAY_TOGGLE = 20,
        SELECT_CLICK,
        TABLE_TOGGLE
    };

    /*
     *  Custom GLV view
     */
    struct L3GLV : glv::GLV
    {
        bool onEvent( glv::Event::t e, glv::GLV& g)
        {
            glv::space_t a =0.0f;
            glv::space_t b =0.0f;

            if ( e == glv::Event::KeyDown )
            {
                const glv::Keyboard& k = g.keyboard();

                switch (k.key())
                {
                    //case 96:
                    case '`':
                        this->broadcastEvent( static_cast< glv::Event::t>( OVERLAY_TOGGLE ) );

                        // This is quite grim
                        this->setMouseDown(a, b, 1, 1);
                        this->setMouseUp(a, b, 1, 1);

                        break;

                    case glv::Key::Tab:
                        this->broadcastEvent( static_cast< glv::Event::t>( TABLE_TOGGLE ) );
                        
                        break;

                    default:
                        break; 
                
                }
            }
        }
    };

    struct  CustomTable : glv::Table
    {

        CustomTable( const char * arrangement="<", glv::space_t padX=3, glv::space_t padY=3, const glv::Rect& r= glv::Rect(0)) 
            : Table( arrangement, padX, padY, r )
        {
              //this->enable( glv::DrawBack | glv::DrawBorder | glv::CropChildren | glv::FocusHighlight | glv::Controllable);
              this->enable( glv::DrawBack | glv::Controllable);
        }

        bool onEvent( glv::Event::t type, glv::GLV& g );
    };

    struct TableToggler : glv::View
    {

        TableToggler( std::deque< boost::shared_ptr< glv::Table > > * tables );
        
        int current_table;
        std::deque< boost::shared_ptr< glv::Table > > * tables;

        bool onEvent( glv::Event::t type, glv::GLV& g );
        
    };

}
}
#endif

