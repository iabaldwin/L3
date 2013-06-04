#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "L3Vis.h"

int main( int argc, char* argv[] )
{

    boost::shared_ptr< L3::Container > container( new L3::Container() );

    glv::Window win(1400, 800, "Visualisation::Estimation");

    L3::Visualisers::EstimatorLayout layout( win );
    
    L3::Interface* command_interface = new L3::CommandInterface( &layout, container );
    L3::Interface* lua_interface = new L3::LuaInterface();

    (*dynamic_cast< L3::Visualisers::GLVInterface* >( layout.scripting_interface.get() ) ) << command_interface << lua_interface;

    layout.run();

}

