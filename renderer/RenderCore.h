#ifndef L3_VISUALISERS_RENDER_CORE_H
#define L3_VISUALISERS_RENDER_CORE_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

namespace L3
{
namespace Visualisers
{
 
enum CUSTOM_EVENT_TYPES
    {
        OVERLAY_TOGGLE = 20,
        SELECT_CLICK 
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
                    case 96:
                        this->broadcastEvent( static_cast< glv::Event::t>( OVERLAY_TOGGLE ) );

                        // This is quite grim
                        this->setMouseDown(a, b, 1, 1);
                        this->setMouseUp(a, b, 1, 1);

                    case 126:
                        //this->broadcastEvent( static_cast< glv::Event::t>( SELECT_CLICK ) );

                    default:
                        break; 
                
                }
            }
            
           
            /*
             *  Select-click
             */
            if ( e == glv::Event::MouseDown )
            {
                const glv::Keyboard& k = g.keyboard();
           
                if ( k.ctrl() )
                    this->broadcastEvent( static_cast< glv::Event::t>( SELECT_CLICK ) );
            }

        }
    };

}
}
#endif

