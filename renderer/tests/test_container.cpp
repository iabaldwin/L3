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
    
    L3::Interface* command_interface = new L3::CommandInterface( &layout, boost::shared_ptr< L3::Container>( new L3::Container() ) );
    L3::Interface* lua = new L3::LuaInterface();

    (*dynamic_cast< L3::Visualisers::GLVInterface* >( layout.scripting_interface.get() ) ) << command_interface << lua;
    
    layout.run();

}

