#include <iostream>

#include "L3.h"

int main()
{
    /*
     *  Dataset
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();
    
    //Dataset configuration
    L3::Configuration::Mission mission( dataset );

    double time = dataset.start_time;

    /*
     *  Experience
     */
    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    L3::ExperienceLoader experience_loader( experience_dataset );

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;
    
    /*
     *  Iterators
     */
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers[ mission.declined] );
    L3::ConstantTimeIterator< L3::SE3 >    pose_iterator( dataset.pose_reader );
    
    pose_iterator.swathe_length = LIDAR_iterator.swathe_length;
    
    L3::ConstantTimePoseWindower    pose_windower( &pose_iterator );
    L3::SwatheBuilder               swathe_builder( &pose_windower, &LIDAR_iterator );

    /*
     *  Projection 
     */
    L3::SE3 projection(0,0,0,.1,.2,.3);
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud) );

    /*
     *  Estimation
     */
    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    L3::Estimator::DiscreteEstimator<double> estimator( kl_cost_function );


    /*
     *  Run
     */
    double increment = 1.0;
 
    L3::EstimatorRunner runner;
    
    runner.setExperience( &*experience )
            .setPoseProvider( &pose_windower )
            .setProjector( &*projector )
            .setEstimator( &estimator  )
            .setSwatheBuilder( &swathe_builder );

    while( true )
    {
        time += increment;

        // Update
        runner.update( time );
    }
}
