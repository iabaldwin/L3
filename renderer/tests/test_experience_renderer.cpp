#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

int main (int argc, char ** argv)
{
    /*
     *L3
     */
    boost::shared_ptr<L3::Experience> experience;
    
    try
    {
        //L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
        L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
        L3::ExperienceLoader experience_loader( dataset );
        experience = experience_loader.experience;
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
    L3::Visualisers::Composite                          composite( glv::Rect(800,800));
    L3::Visualisers::CompositeController                controller( &composite, composite.position );
    boost::shared_ptr< L3::PoseProvider >               pose_provider( new L3::CircularPoseProvider( 25.0 ) );
    L3::Visualisers::PoseProviderRenderer               pose_provider_renderer( pose_provider.get() );
    L3::Visualisers::ExperienceRenderer                 experience_renderer( experience );
    L3::Visualisers::HistogramPyramidRendererView       pyramid_renderer( experience->experience_pyramid, 3 );

    L3::Visualisers::ExperienceLocationOverviewView     experience_view( glv::Rect(250,250), experience );

    experience_view.pos( win.width() - 400, win.height() - 350 );

    experience_view.provider = pose_provider;

    pyramid_renderer.pos( win.width()- (180*3), 0 );

    L3::Visualisers::Updater updater;
    updater << &pyramid_renderer;

    // Associate pose provider
    experience_renderer.addPoseProvider( pose_provider.get() );

    // Combine
    top << (composite << grid << experience_renderer << pose_provider_renderer ) << pyramid_renderer << updater << experience_view;

    // Run
    win.setGLV(top);
    glv::Application::run();

}
