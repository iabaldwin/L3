#include <iostream>
#include "renderer.h"

struct Scene : glv::View3D{

    Scene()
    {
        stretch(1,1); 
    }
	
	virtual void onDraw3D(glv::GLV& g)
    {
    
    }

};
