#ifndef L3_EXTERNAL_INTERFACE_H
#define L3_EXTERNAL_INTERFACE_H

#include <GLV/glv.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "L3.h"

#include "RenderCore.h"
#include "Interface.h"

namespace L3
{
    namespace Visualisers
    {
        struct GLVInterface : glv::TextView
        {
            GLVInterface( L3GLV* master ) ;

            L3GLV* master;
            bool visibility;
            std::list< L3::Interface* > interfaces;
            std::list < std::string > full_history;
            std::deque< std::string > command_history;
                                
            bool onEvent( glv::Event::t e, glv::GLV& g);

            bool handleText( glv::Event::t e, glv::GLV& g);

            GLVInterface& operator<< ( L3::Interface* iface )
            {
                interfaces.push_front( iface );
                return *this;
            }
       
            void dumpHistory();
    
            void selectAll();
        };

    }
}

#endif

