#include <iostream>

#include "L3.h"

int main()
{
    /*
     *Load dataset
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();
    
    L3::Configuration::Mission mission( dataset );


    /*
     *Load experience
     */
    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    L3::ExperienceLoader experience_loader( experience_dataset );

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;
    /*
     *Constant time iterator over poses
     */
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers[ mission.declined] );
    L3::ConstantTimeIterator< L3::SE3 >    pose_iterator( dataset.pose_reader );
    
    pose_iterator.swathe_length = LIDAR_iterator.swathe_length;
    
    double time = dataset.start_time;

    L3::ConstantTimePoseWindower pose_windower( &pose_iterator );
    
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    /*
     * Projector  
     */
    L3::SE3 projection(0,0,0,.1,.2,.3);
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud) );

    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    L3::Estimator::DiscreteEstimator<double> estimator( kl_cost_function );



    /*
     * Run
     */
    L3::Tools::Timer t;
    double increment = 1.0;
        
    boost::shared_ptr<L3::PointCloud<double> > experience_cloud;
    while( true )
    {
        t.begin();
        
        time += increment;
       
        pose_windower.update( time );

        experience->update( rand()%100, rand()%100 );

        if ( !swathe_builder.update( time ))
            throw std::exception();

        projector->project( swathe_builder.swathe );

        experience->getExperienceCloud( experience_cloud );

        estimator( &*experience_cloud, point_cloud, L3::SE3::ZERO() );

        double end_time = t.end(); 
        //std::cout << cloud.size() << " pts\t " << end_time << "s" << "\t" << "[" << (double)cloud.size()/end_time << " pts/s" << "]" << " :" << swathe_builder.window_duration << std::endl;
        std::cout << end_time << std::endl;
    
    }
}
