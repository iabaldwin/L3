#ifndef L3_CONTROLLERS_H
#define L3_CONTROLLERS_H

#include <GLV/glv.h>

#include "L3.h"


namespace L3
{
namespace Visualisers
{

struct Controller : glv::EventHandler
{

    bool onEvent( glv::View& v, glv::GLV& g )
    {
        if(g.keyboard().key() == glv::Key::Escape) 
        {   
            g.printDescendents();
           
            L3::Visualisers::Composite* ptr = dynamic_cast<L3::Visualisers::Composite*>( g.child );

            if ( ptr )
                ptr->move( 0, 0, 1 );
            return true;
        }
    }

};


}
}


#endif
