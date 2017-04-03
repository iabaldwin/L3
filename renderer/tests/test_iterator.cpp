#include <iostream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

#include <boost/scoped_ptr.hpp>


struct Translator : L3::Visualisers::Leaf
{

    Translator( boost::shared_ptr< L3::ConstantTimeIterator< L3::SE3 > > ptr ) : ptr(ptr)
    {

    }

    boost::weak_ptr< L3::ConstantTimeIterator< L3::SE3 > > ptr;

    void onDraw3D( glv::GLV& g )
    {
    
        boost::shared_ptr< L3::ConstantTimeIterator< L3::SE3 > > it_ptr = ptr.lock();

        glv::draw::translate( -1*it_ptr->window.back().second->X(),
                                -1*it_ptr->window.back().second->Y(),
                                -50 );
    }

};

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
    L3::Visualisers::CompositeController        controller( &composite, composite.position );

    Translator t( iterator );

    L3::Visualisers::VisualiserRunner runner( dataset->start_time );
    runner << iterator.get();

    composite.stretch(1,1);
    
    top << ( composite << grid << runner << t << iterator_renderer );

    win.setGLV(top);
    glv::Application::run();

}


