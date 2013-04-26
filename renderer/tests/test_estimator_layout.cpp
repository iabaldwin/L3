#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Layouts.h"

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
    L3::Dataset dataset( dataset_directory );
   
    if( !( dataset.validate() && dataset.load() ) )
        exit(-1);

    // Configuration
    L3::Configuration::Mission mission( dataset );

    // Experience
    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    //L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >     oracle_source( dataset.pose_reader );
    L3::ConstantTimeIterator< L3::LHLV >    integrated_pose_iterator( dataset.LHLV_reader );

    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 >  vertical_LIDAR_iterator( dataset.LIDAR_readers[ mission.declined ] );
    L3::ConstantTimeIterator< L3::LMS151 >  horizontal_LIDAR_iterator( dataset.LIDAR_readers[ mission.horizontal ] );
  
    // Pose Windower
    L3::ConstantTimeWindower<L3::SE3>   oracle( &oracle_source);
    L3::ConstantTimeWindower<L3::LHLV>  pose_windower( &integrated_pose_iterator );
    
    // Swathe builder
    L3::SwatheBuilder swathe_builder( &pose_windower, &vertical_LIDAR_iterator );

    // Projection
    boost::shared_ptr< L3::PointCloud<double> > point_cloud = boost::make_shared<L3::PointCloud<double> >();
    L3::SE3 projection = L3::SE3::ZERO();
   
    L3::Configuration::convert( mission.lidars[ mission.declined], projection );
    
    boost::shared_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud.get() ) );

    // Estimator
    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    L3::Estimator::DiscreteEstimator<double> estimator( kl_cost_function, experience->experience_histogram );

    // Create runner
    L3::EstimatorRunner runner;

    // Updateables
    runner << &vertical_LIDAR_iterator << &horizontal_LIDAR_iterator << &pose_windower;

    runner.setExperience( &*experience )
          .setPoseWindower( &pose_windower )
          .setPoseProvider( &oracle )
          .setProjector( &*projector )
          .setEstimator( &estimator  )
          .setSwatheBuilder( &swathe_builder )
          .setHorizontalLIDAR( &horizontal_LIDAR_iterator )
          .setVerticalLIDAR( &vertical_LIDAR_iterator )
          .start( dataset.start_time );

    glv::Window win(1400, 800, "Visualisation::EstimatorLayout");

    //L3::Visualisers::EstimatorLayout layout(win, &runner, experience, point_cloud.get() );
    L3::Visualisers::EstimatorLayout layout(win, &runner, experience, point_cloud );

    glv::GLV top;

    layout.run( top );
}

