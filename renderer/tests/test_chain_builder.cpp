#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"

int main()
{
    try
    {
        L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

        if( !(dataset.validate() && dataset.load() ) )
            throw std::exception();

        // Constant time iterator over LHLV data
        L3::ConstantTimeIterator< L3::LHLV > iterator( dataset.LHLV_reader, 20.0 );

        double time = dataset.start_time;

        std::cout.precision(15);

        L3::ChainBuilder builder( &iterator );

        L3::Tools::Timer t;

        std::pair<double,int> latency( 0.0, 0 );

        // Run
        while (true)
        {
            usleep( .1*1e6 );
            t.begin();
            if ( !builder.update( time += 1 ) )
                throw std::exception();

            std::cout << time << "-->" << iterator.window.front().first << ":" << iterator.window.back().first << ":" << iterator.window.back().first - iterator.window.front().first <<  "(" << iterator.window.size() << ")" << std::endl;

        } 

        /*
         *Visualisation
         */
        glv::GLV top;
        glv::Window win(1400, 800, "Visualisation::PointCloud");

        // Colors
        top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

        // Point cloud renderer
        L3::Visualisers::Composite          composite;
        L3::Visualisers::BasicPanController controller;
        L3::Visualisers::Grid               grid;
        L3::Visualisers::SwatheRenderer     swathe_renderer( &swathe_builder ); 

        composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );
        composite.current_time = time;
        composite.sf = 2.0;

        top << (composite << swathe_renderer << grid );

        win.setGLV(top);
        win.fit(); 
        glv::Application::run();

    }
    catch( L3::no_such_folder& e )
    {
        std::cerr << "No such folder" << std::endl;
    }

}
