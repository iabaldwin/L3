#include <iostream>

#include "L3.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Load experience
    L3::Dataset experience_dataset(  "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    // Constant time iterator over:
    // 1. Poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    pose_iterator.swathe_length = 60;
    // 2. Poses
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.begin()->second );
    LIDAR_iterator.swathe_length = 60;
    
    double time = dataset.start_time;

    // Pose windower
    L3::ConstantTimePoseWindower pose_windower( &pose_iterator );
    
    // Swathe builder
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    // Projection
    L3::SE3 projection(0,0,0,.1,.2,.3);
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud ) );

    // Histogrammer
    L3::Histogrammer<double> histogrammer( NULL, NULL );

    L3::Tools::Timer t;

    // Run
    double increment = .02;
    while (true)
    {
        usleep( increment*1e6 );
        t.begin();
       
        // Build the swathe
        swathe_builder.update( time += increment ) ;
       
        experience->update( rand()%100, rand()%100 );
      
        boost::shared_ptr<L3::PointCloud<double> > experience_cloud;

        if( !experience->getExperienceCloud( experience_cloud ))
            throw std::exception();

        projector->project( swathe_builder.swathe );
        
        histogrammer.experience = &*experience_cloud;
        histogrammer.swathe = point_cloud;

        histogrammer.histogram();

        std::cout << __PRETTY_FUNCTION__ << ":" << t.end() << "\t(" << swathe_builder.swathe.size() << ")" << std::endl;
    } 
}
