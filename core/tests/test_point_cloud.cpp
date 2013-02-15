#include <iostream>
#include <vector>

#include "Iterator.h"
#include "Utils.h"
#include "Dataset.h"
#include "Projector.h"
#include "Pointcloud.h"

template <typename T>
L3::Point<T> randomate()
{
    return L3::Point<T>( random() % 100, random() % 100, random() % 100  );
}

int main()
{
    // Build cloud
    std::auto_ptr<L3::PointCloudXYZ<double> > cloud ( new L3::PointCloudXYZ<double>() );

    std::vector< L3::Point<double> > randoms(2*1000000);

    std::generate( randoms.begin(), randoms.end(), randomate<double> );

    std::copy( randoms.begin(), randoms.end(), std::back_inserter( cloud->data) );
    
    //std::cout << *cloud << std::endl;

    L3::Tools::Timer t;
    t.begin();
    cloud->histogram();
    double elapsed = t.end();

    std::cout << cloud->size() << " pts histogrammed in " << elapsed << std::endl;

    for ( int i = 0; i<1000; i++ )
    {
        t.begin();
        L3::SE3 pose( random()%100, random()%100, random()%100, (random()%10)/1000, (random()%10)/1000, (random()%10)/100);
        cloud->transform( &pose );
        elapsed = t.end();
        std::cout << cloud->size() << " pts translated/rotated in " << elapsed << std::endl;
    }

}
