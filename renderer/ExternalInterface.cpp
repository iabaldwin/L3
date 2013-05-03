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
                //this->disable( glv::DrawBack );
                this->disable( glv::Focused );
                //this->disable( glv::HitTest );
                //this->disable( glv::FocusToTop );
                //this->disable( glv::DrawBack );
                //this->disable( glv::Controllable );
                //this->disable( glv::Animate );
                //this->disable( glv::FocusHighlight );
                return false;
            }
            else
            {
                this->enable( glv::Focused );
                this->maximize();
                //this->bringToFront();
                this->focused(true);
                //this->enable( glv::Focused );
                //this->enable( glv::DrawBack );
                //setPos(xToPos(0));
                //cursorPos(0);
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
                //setPos(0);
                cursorPos(0);

                std::stringstream ss; 
                for ( std::list<std::string>::iterator it=history.begin(); it != history.end(); it++ )
                    ss << *it;

                std::cout << ss.str();
                std::cout << "--------------" << std::endl;
    
                unsigned found = current.find( "p\"" );
                   
                //if ( found !=std::string::npos )
                //{
                    //current.erase( found,1 );
                //}

                std::cout << current << std::endl;

                //std::cout <<interface->execute( current ) << std::endl;
            }
        }

        return retval;

        //return glv::View::onEvent( e,g );
    }
}
}
