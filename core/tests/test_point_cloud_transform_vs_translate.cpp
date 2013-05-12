#include <iostream>
#include <vector>

#include "Utils.h"
#include "Dataset.h"
#include "Projector.h"
#include "PointCloud.h"
#include "Histogram.h"

template <typename T>
L3::Point<T> randomate()
{
    return L3::Point<T>( random() % 100, random() % 100, random() % 100  );
}

int main()
{
    /*
     *Build cloud
     */
    L3::PointCloud<double>*  cloud = new L3::PointCloud<double>();

    std::vector< L3::Point<double> > randoms(20*100000);

    std::generate( randoms.begin(), randoms.end(), randomate<double> );

    cloud->points = &randoms[0];
    cloud->num_points = randoms.size();

    L3::Timing::SysTimer t;

    double elapsed;

    for ( int i = 0; i<1000; i++ )
    {
        L3::SE3 pose( random()%100, random()%100, random()%100, (random()%10)/1000, (random()%10)/1000, (random()%10)/100);
        t.begin();
        L3::transform( cloud, &pose );
        elapsed = t.elapsed();
        
        std::cout << cloud->num_points << " pts rotated in \t\t" << elapsed << std::endl;
        
        t.begin();
        L3::translate( cloud, &pose );
        elapsed = t.elapsed();
        
        
        std::cout << cloud->num_points << " pts translated in \t" << elapsed << std::endl;
    }

}
