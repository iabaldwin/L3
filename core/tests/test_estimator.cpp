#include <iostream>
#include <fstream>

#include <readline/readline.h>

#include "L3.h"

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
    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;
    

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >     oracle_source( dataset.pose_reader );
    L3::ConstantTimeIterator< L3::LHLV >    integrated_pose_iterator( dataset.LHLV_reader );

    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 >  LIDAR_iterator( dataset.LIDAR_readers[ mission.declined ] );
  
    // Pose Windower
    L3::ConstantTimeWindower<L3::SE3>   oracle( &oracle_source);
    L3::ConstantTimeWindower<L3::LHLV>  pose_windower( &integrated_pose_iterator );
    
    // Swathe builder
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    // Projection
    L3::SE3 projection(0,0,0,.1,.2,.3);
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    boost::shared_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud) );

    // Estimator
    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    //L3::Estimator::DiscreteEstimator<double> estimator( kl_cost_function, (*experience->experience_pyramid)[0] );
    L3::Estimator::IterativeDescent<double> estimator( kl_cost_function, experience->experience_pyramid );

    // Create runner
    L3::EstimatorRunner runner;

    runner << &LIDAR_iterator << &pose_windower;

    runner.setExperience( &*experience )
          .setPoseWindower( &pose_windower )
          .setPoseProvider( &oracle )
          .setProjector( &*projector )
          .setAlgorithm( &estimator  )
          .setSwatheBuilder( &swathe_builder )
          .start( dataset.start_time );

    while( true )
    {
        //usleep( 1*1e6 ); 
   
        char* res = readline( ">> " );
   
        if ( !res )
            break;
    }
}

