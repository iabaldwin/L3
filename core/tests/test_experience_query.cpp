#include "L3.h"


int main()
{
    // Build experience
    L3::ExperienceLoader experience_loader;

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    for ( int i=0; i<100; i++ )
        try
        {
            experience->load( i );
            experience->update( random()%100, random()%100 );
        }
        catch(...)
        {
        }

}
