#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"

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
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::ExperienceRenderer");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    // Point cloud renderer
    L3::Visualisers::Composite              composite;
    L3::Visualisers::Controller*            controller = new L3::Visualisers::BasicPanController();
    L3::Visualisers::Grid                   grid;
    L3::PoseProvider*                       pose_provider = new L3::CircularPoseProvider();
    L3::Visualisers::PoseProviderRenderer   pose_provider_renderer( pose_provider );
    L3::Visualisers::ExperienceRenderer     experience_renderer( experience );

    // Associate pose provider
    experience_renderer.addPoseProvider( pose_provider );

    // Associate controller
    composite.addController( controller );
   
    // Combine
    top << (composite << grid << experience_renderer << pose_provider_renderer ) ;

    // Run
    win.setGLV(top);
    win.fit(); 
    glv::Application::run();

}
