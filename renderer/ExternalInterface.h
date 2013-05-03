#ifndef L3_EXTERNAL_INTERFAE_H
#define L3_EXTERNAL_INTERFAE_H

#include <GLV/glv.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "Interface.h"

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
           
            // Always bubble events to top-level
            this->enable( glv::AlwaysBubble );

            interface.reset( new L3::LuaInterface() );
        }

        bool visibility;

        std::list < std::string > history;

        bool onEvent( glv::Event::t e, glv::GLV& g);

        boost::shared_ptr< L3::Interface > interface;
    };

}
}

#endif

