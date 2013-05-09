#ifndef L3_EXTERNAL_INTERFACE_H
#define L3_EXTERNAL_INTERFACE_H

#include <GLV/glv.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "L3.h"

#include "Interface.h"
#include "L3Interface.h"

namespace L3
{
    namespace Visualisers
    {
        struct GLVInterface : glv::TextView
        {
            GLVInterface( glv::Rect rect ) : glv::TextView( rect )
            {
                visibility = false;

                // Full screen, but not visible
                this->maximize(); 
                this->disable(glv::Visible);

                // Always bubble events to top-level, so we can catch toggle
                this->enable( glv::AlwaysBubble );

                // Initialisze
                mText = ">> ";
                cursorPos(3);
            }

            bool visibility;
            std::list < std::string > history;
            std::list< L3::Interface* > interfaces;

            bool onEvent( glv::Event::t e, glv::GLV& g);

            bool handleText( glv::Event::t e, glv::GLV& g);

            GLVInterface& operator<< ( L3::Interface* iface )
            {
                interfaces.push_front( iface );
                return *this;
            }
        };

    }
}

#endif

