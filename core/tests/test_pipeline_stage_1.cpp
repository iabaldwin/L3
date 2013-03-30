#include "L3.h"

int main()
{
    
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    
    // Load experience
    L3::ExperienceLoader experience_loader( dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.back() );
    
    double time = dataset.start_time;

    L3::ConstantTimePoseWindower pose_windower( &pose_iterator );
    
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );


    L3::Tools::Timer t;

    // Run
    double increment = .02;
    while (true)
    {
        t.begin();
        swathe_builder.update( time += increment ) ;
        std::cout << __PRETTY_FUNCTION__ << ":" << t.end() << "\t(" << swathe_builder.swathe.size() << ")" << std::endl;
    } 
}
