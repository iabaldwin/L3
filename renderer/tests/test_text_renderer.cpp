#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

struct tmp : L3::Visualisers::LabelledLeaf
{

    tmp()
    {
        this->label.reset( new L3::Visualisers::LeafLabel( &this->tag ) );
    }
    
    boost::shared_ptr< L3::Visualisers::LeafLabel > label;

    void onDraw3D( glv::GLV& g )
    {
        
        std::stringstream ss;

        this->tag.x = 0;
        this->tag.y = 0;
        ss << "Hello, World";
        this->tag.text = ss.str();

        label->onDraw3D(g);
    }

};


int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite composite;
    L3::Visualisers::BasicPanController         controller(composite.position);
    L3::Visualisers::Grid                       grid;

    tmp tmp_leaf;

    top << ( composite << tmp_leaf << grid );

    composite.stretch(1,1);

    win.setGLV(top);
    glv::Application::run();
}


