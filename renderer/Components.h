#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>
#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>


namespace L3
{

struct Component : glv::View3D{

    Component()
    {
        stretch(1,1); 
    }
	
	virtual void onDraw3D(glv::GLV& g)
    {
    
    }

};

struct PoseChain : Component
{

    PoseChain()
    {

    }

    virtual void onDraw3D(glv::GLV& g)
    {
    }

};

}
#endif
