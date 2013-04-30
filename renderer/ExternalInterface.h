#ifndef L3_EXTERNAL_INTERFAE_H
#define L3_EXTERNAL_INTERFAE_H

#include <boost/bind.hpp>

namespace L3
{
namespace Visualisers
{

    struct ExternalInterface : glv::TextView 
    {
        ExternalInterface( glv::Rect rect ) : glv::TextView( rect )
        {
            visible = false;
            this->maximize(); 
            this->disable(glv::Visible);
        }

        bool visible;

        std::list < std::string > history;

        bool onEvent( glv::Event::t e, glv::GLV& g)
        {
            if ( e==20 )
            {
                visible ? this->disable(glv::Visible) : this->enable(glv::Visible);
                visible = !visible; 
                if (visible )
                {
                    this->enable( glv::Focused );
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
                    history.push_front(  this->getValue() + "\n" );

                    deleteText(0, mText.size());
                    setPos(0);
 
                    std::stringstream ss; 
                    for ( std::list<std::string>::iterator it=history.begin(); it != history.end(); it++ )
                        ss << *it;

                    std::cout << ss.str() << std::endl;
                    std::cout << "--------------" << std::endl;
                }
            }

        }

    };

}
}

#endif

