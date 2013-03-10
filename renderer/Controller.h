#ifndef L3_CONTROLLERS_H
#define L3_CONTROLLERS_H

#include <GLV/glv.h>

#include "L3.h"

#include <GLUT/glut.h>


namespace L3
{
namespace Visualisers
{

struct Controller : glv::EventHandler
{
    Controller()
    {
        glutIgnoreKeyRepeat(0);
    }

    bool onEvent( glv::View& v, glv::GLV& g )
    {
        L3::Visualisers::Composite* ptr = dynamic_cast<L3::Visualisers::Composite*>( g.child );

        if (!ptr)
            return false;

        double x=0, y=0, z=0, r=0, p=0, q =0;

        switch(g.keyboard().key())
        {
            case 119:   // w
                z+=1;
                break;

            case 115:   // s
                z-=1;
                break;

            case 97:    // a
                x+=1;
                break;

            case 100:   // d
                x-=1;
                break;

            default:
                break;
        }
                
        ptr->move( x,y,z);
        ptr->rotate( r,p,q);

        return true;

    }

};


}
}


#endif
