#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "L3Vis.h"

int main()
{
    glv::Window win(1400, 800, "Visualisation::Estimator");

    L3::Visualisers::EstimatorLayout layout( win );
    
    L3::Container container( &layout );

    L3::Interface* command_interface = new L3::CommandInterface( &container );

    (*dynamic_cast< L3::Visualisers::GLVInterface* >( layout.scripting_interface.get() ) ) << command_interface;
    
    layout.run();
}

