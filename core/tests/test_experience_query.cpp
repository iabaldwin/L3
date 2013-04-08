#include "L3.h"


int main()
{
    // Load datset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );

    // Load experience
    L3::ExperienceLoader experience_loader( dataset );

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    L3::Tools::Timer t;
    for ( int i=0; i<100; i-- )
    {
        experience->update( random()%100, random()%100 );
    
        t.begin();
        L3::ReadLock( experience->experience_histogram->mutex );
        std::cout << experience->experience_histogram->x_bins<< " x" << experience->experience_histogram->y_bins << " in " << t.end() << "s" << std::endl;
    }

}
