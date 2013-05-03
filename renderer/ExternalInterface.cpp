#include "ExternalInterface.h"
#include <iostream>
#include <sstream>

namespace L3
{
namespace Visualisers
{
    bool ExternalInterface::onEvent( glv::Event::t e, glv::GLV& g)
    {
        if ( e==20 )
        {
            visibility ? this->disable(glv::Visible) : this->enable(glv::Visible);
            visibility = !visibility; 
  
            if (!visibility )
            {
                this->restore(); 
                this->focused(false);
                this->disable( glv::Focused );
                this->disable( glv::DrawBack );
                this->disable( glv::HitTest );
                return false;
            }
            else
            {
                this->maximize();
                this->focused(true);
                this->bringToFront();
                this->enable( glv::Focused );
                this->enable( glv::DrawBack );
                this->enable( glv::HitTest );
            }
        }

        const glv::Keyboard& k = g.keyboard();
        int key = k.key();
        float mx = g.mouse().xRel();

        // Intercept switch
        switch(e){
            case glv::Event::KeyDown:
            if( key == 96 ){
                return false;
			}
	
        }

        // Process  
        bool retval = true;
        retval = retval && glv::TextView::onEvent(e,g);

        if ( e == glv::Event::KeyDown) 
        {
            const glv::Keyboard& k = g.keyboard();
            int key = k.key();

            if ( key == glv::Key::Return )
            {
                std::string current = this->getValue() ;
                history.push_front(  current + "\n" );

                deleteText(0, mText.size());
                cursorPos(0);

                std::stringstream ss; 
                for ( std::list<std::string>::iterator it=history.begin(); it != history.end(); it++ )
                    ss << *it;

                std::cout << ss.str();
                std::cout << "--------------" << std::endl;
    
            }
        }

        return retval;
    }
}
}
