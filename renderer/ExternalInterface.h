#ifndef L3_EXTERNAL_INTERFAE_H
#define L3_EXTERNAL_INTERFAE_H

#include <GLV/glv.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "Interface.h"
#include "L3Interface.h"

namespace L3
{
namespace Visualisers
{

    struct ExternalInterface : glv::TextView
    {
        ExternalInterface( glv::Rect rect ) : glv::TextView( rect )
        {
            visibility = false;
            
            this->maximize(); 
            this->disable(glv::Visible);
           
            // Always bubble events to top-level, so we can catch toggle
            this->enable( glv::AlwaysBubble );

            // Create interface
            interface.reset( new L3::LuaInterface() );
            L3_interface.reset( new L3::L3Interface() );

            // Initialisze
            mText = ">> ";
            cursorPos(3);

        }
    
        bool visibility;
        std::list < std::string > history;
        boost::shared_ptr< L3::Interface > interface;
        boost::shared_ptr< L3::Interface > L3_interface;

        bool onEvent( glv::Event::t e, glv::GLV& g);

        bool handleText( glv::Event::t e, glv::GLV& g);

    };

}
}

#endif

