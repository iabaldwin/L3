#include <iostream>
#include "renderer.h"

struct Renderer : glv::View3D{

    Renderer()
    {
        stretch(1,1); 
    }
	
	virtual void onDraw3D(glv::GLV& g)
    {
    
    }

};
