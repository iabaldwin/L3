#include <iostream>
#include "Dataset.h"
#include "Tools.h"

int main()
{
    L3::Dataset d( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    assert( d.validate() && d.load() );

    std::cout.precision( 20 );

    L3::Tools::Timer t;

    for( int i=0; i<100; i++ )
    {
        int index = random() % d.poses.size(); 
        double time = d.poses[index]->time;
        
        t.begin();
        L3::Pose* p =  d.getPoseAtTime( time  );
        //std ::cout << t.end() << std::endl;
        printf( "%.5f\n", t.end() );

        L3::LMS151* s = d.getScanAtTime( p->time, d.LIDAR_names[0] );

        //std::cout << p->time << std::endl;
        //std::cout << s->time << std::endl;
        //std::cout << p->time - s->time << std::endl;

        //std::cout << p->time - d.poses[0]->time << std::endl;

        //std::cout << index << std::endl;
        //std::cout << std::distance( d.poses.begin(), std::find( d.poses.begin(), d.poses.end(), p ) ) << std::endl;

        assert( index == std::distance( d.poses.begin(), std::find( d.poses.begin(), d.poses.end(), p ) ) ); 

    }

}
