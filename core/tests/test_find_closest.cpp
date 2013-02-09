#include <iostream>
#include "Dataset.h"

int main()
{
    L3::Dataset d( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    d.validate();
    d.load();

    std::cout.precision( 20 );

    //for ( std::vector<L3::Pose*>::iterator it = d.poses.begin(); it != d.poses.end(); it++ )
        //std::cout << (*it)->time << std::endl;

    double time = d.poses[20]->time;
    L3::Pose* p =  d.getPoseAtTime( time  );

    std::cout << time << std::endl;
    std::cout << p->time << std::endl;
    std::cout << d.poses[0]->time << std::endl;

    //L3::LMS151* s = d.getScanAtTime( p->time, d.LIDAR_names[0] );
    //std::cout << s << std::endl;

    //std::cout << p->time << std::endl;
    //std::cout << s->time << std::endl;
    //std::cout << p->time - s->time << std::endl;

    //std::cout << p->time - d.poses[0]->time << std::endl;
    //std::cout << d.poses[20]->time << std::endl;
}

