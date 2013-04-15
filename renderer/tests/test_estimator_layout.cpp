#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Layouts.h"

int main ()
{
 
    /*
     *  L3
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    if( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Configuration
    L3::Configuration::Mission mission( dataset );

    // Experience
    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    
    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers[ mission.declined ] );
  
    // Pose Windower
    L3::ConstantTimePoseWindower pose_windower( &pose_iterator );
    
    // Swathe builder
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    // Projection
    L3::SE3 projection(0,0,0,.1,.2,.3);
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud) );

    // Estimator
    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    L3::Estimator::DiscreteEstimator<double> estimator( kl_cost_function, experience->experience_histogram );

    // Create runner
    L3::EstimatorRunner runner;
    
    runner.setExperience( &*experience )
          .setPoseProvider( &pose_windower )
          .setProjector( &*projector )
          .setEstimator( &estimator  )
          .setSwatheBuilder( &swathe_builder )
          .start( dataset.start_time );

    glv::Window win(1400, 800, "Visualisation::Layout");

    L3::Visualisers::EstimatorLayout layout(win);

    glv::GLV top;

    layout.load( &runner );

    layout.run( top );

}
