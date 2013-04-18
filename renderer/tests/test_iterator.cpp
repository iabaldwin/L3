#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"


int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Viewer::Iterator");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Pose sequence
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();
    
    L3::ConstantTimeIterator< L3::SE3 > iterator( dataset.pose_reader );

    L3::Visualisers::Grid                       grid;
    L3::Visualisers::Composite                  composite;
    L3::Visualisers::IteratorRenderer<L3::SE3>  iterator_renderer( &iterator  );

    L3::Visualisers::VisualiserRunner runner( dataset.start_time );

    runner << &iterator;

    composite << iterator_renderer << grid << runner;

    top << composite;

    win.setGLV(top);
    glv::Application::run();

}


