#include <iostream>
#include <fstream>

#include "L3.h"
#include "L3Vis.h"

int main( int argc, char* argv[] )
{

    if ( argc != 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <dataset>" << std::endl;
        exit(-1);
    }

    char* dataset_directory = argv[1];
 
    /*
     *  L3
     */
    L3::Dataset* dataset = new L3::Dataset( dataset_directory );
   
    if( !( dataset->validate() && dataset->load() ) )
        exit(-1);

    // Configuration
    L3::Configuration::Mission* mission = new L3::Configuration::Mission( *dataset ) ;

    L3::DatasetRunner* runner = new L3::DatasetRunner( dataset, mission );
    runner->start();
  
    boost::shared_ptr< L3::Container > container( new L3::Container() );

    container->dataset    = boost::shared_ptr< L3::Dataset>( dataset );
    container->runner     = boost::shared_ptr< L3::DatasetRunner >(runner);
    container->mission    = boost::shared_ptr< L3::Configuration::Mission >( mission );

    glv::Window win(1400, 800, "Visualisation::DatasetLayout");
   

    L3::Visualisers::DatasetLayout layout(win);

    L3::Interface* command_interface = new L3::CommandInterface( &layout, container );
    L3::Interface* lua_interface = new L3::LuaInterface();

    (*dynamic_cast< L3::Visualisers::GLVInterface* >( layout.scripting_interface.get() ) ) << command_interface << lua_interface;

    layout.load( runner );

    layout.run();


}
