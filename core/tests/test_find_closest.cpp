#include <iostream>
#include "Dataset.h"

int main()
{
    L3::Dataset d( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    d.validate();
    d.load();
    //if( !( d.validate() && d.load() ) )
        //std::cout << "err" << std::endl;

    //std::cout << d..size() << std::endl;

    //std::cout << d.LIDAR_names[0] << std::endl;

    d.getPoseAtTime( d.poses[0]->time, d.LIDAR_names[0] );

}

