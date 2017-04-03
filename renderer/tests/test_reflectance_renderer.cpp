#include <iostream>
#include <fstream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

int main (int argc, char ** argv)
{
    /*
     *L3
     */
    boost::shared_ptr<L3::Reflectance> reflectance;
    
    try
    {
        //L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
        L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
        L3::ReflectanceLoader experience_loader( dataset );
        reflectance = experience_loader.reflectance;
    }
    catch( std::exception e )
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    /*
     *  Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::ExperienceRenderer");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    // Point cloud renderer
    L3::Visualisers::Grid                               grid;
    L3::Visualisers::Composite                          composite;
    L3::Visualisers::CompositeController                controller( &composite, composite.position );
    
    boost::shared_ptr< L3::PoseProvider >               pose_provider( new L3::CircularPoseProvider( 25.0 ) );
    L3::Visualisers::PoseProviderRenderer               pose_provider_renderer( pose_provider.get() );
    L3::Visualisers::ReflectanceRenderer                reflectance_renderer( reflectance );

    // Associate pose provider
    //reflectance_renderer.addPoseProvider( pose_provider.get() );
    reflectance_renderer.pose_provider = pose_provider;

    // Combine
    L3::Visualisers::VisualUpdater updater;
    top << (composite << grid << reflectance_renderer << pose_provider_renderer ) << updater;
    
    composite.stretch(1,1);

    // Run
    win.setGLV(top);
    glv::Application::run();

}
