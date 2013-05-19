#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"
#include "Imagery.h"
#include "Controls.h"
#include "QueryInterface.h"

int main (int argc, char ** argv)
{

    L3::Visualisers::L3GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Grid                   grid;
    L3::Visualisers::Composite              composite;
    L3::Visualisers::CompositeController    controller( &composite, composite.position );

    L3::Configuration::Begbroke begbroke;
    begbroke.loadDatum();

    boost::shared_ptr< L3::Visualisers::LocaleRenderer > locale_renderer = L3::Visualisers::LocaleRendererFactory::buildMap( begbroke );

    //L3::Visualisers::MouseQuerySelect query(  &composite );
    //L3::Visualisers::WASDManager selection_manager( &query );

    top << ( composite << grid << *locale_renderer  );

    composite.stretch(1,1);

    win.setGLV(top);
    glv::Application::run();
}


