#include <iostream>

#include "L3.h"

int main()
{
    L3::Dataset d( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    d.validate();

    // Build experience
    L3::ExperienceGenerator e( d );
           
    // Sample
    L3::PointCloud<double> sampled_cloud = L3::samplePointCloud( e.point_cloud, 100000 );
    L3::writePCLASCII( "test.pcd", sampled_cloud );
    
    //L3::writePCLASCII( "test.pcd", *e.point_cloud );

}
