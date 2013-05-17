#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

#include <boost/scoped_ptr.hpp>

int main (int argc, char ** argv)
{

    /*
     *  L3
     */
    if ( argc != 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <dataset>" << std::endl;
        exit(-1);
    }

    char* dataset_directory = argv[1];
 
    // Pose sequence
    boost::scoped_ptr< L3::Dataset > dataset( new L3::Dataset( dataset_directory ) );
    if ( !( dataset->validate() && dataset->load() ) )
        exit(-1);

    glv::GLV top;
    glv::Window win(1400, 800, "Viewer::Iterator");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
        
    boost::shared_ptr< L3::ConstantTimeIterator< L3::SE3 > > iterator( new L3::ConstantTimeIterator< L3::SE3 >( dataset->pose_reader ) );

    L3::Visualisers::Grid                       grid;
    L3::Visualisers::Composite                  composite;
    L3::Visualisers::IteratorRenderer<L3::SE3>  iterator_renderer( iterator  );
    L3::Visualisers::BasicPanController         controller(composite.position);

    L3::Visualisers::VisualiserRunner runner( dataset->start_time );
    runner << iterator.get();

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);
    
    top << ( composite << grid << iterator_renderer << runner  );

    win.setGLV(top);
    glv::Application::run();

}


