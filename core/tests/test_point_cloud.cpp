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

    std::vector< L3::Point<double> > randoms(2*100000);

    std::generate( randoms.begin(), randoms.end(), randomate<double> );

    cloud->points = &randoms[0];
    cloud->num_points = randoms.size();

    //std::cout << *cloud << std::endl;

    L3::Tools::Timer t;
    t.begin();
    L3::Histogram<double> hist;
        hist( cloud );
    double elapsed = t.end();

    std::cout << cloud->num_points << " pts histogrammed in " << elapsed << std::endl;

    //for ( int i = 0; i<1000; i++ )
    //{
        //t.begin();
        //L3::SE3 pose( random()%100, random()%100, random()%100, (random()%10)/1000, (random()%10)/1000, (random()%10)/100);
        //cloud->transform( &pose );
        //elapsed = t.end();
        //std::cout << cloud->size() << " pts translated/rotated in " << elapsed << std::endl;
    //}

    //t.begin();
    //L3::PointCloud<double> sampled = L3::samplePointCloud( *cloud, 10000000 );
    //std::cout << sampled.size() << " pts resampled in " << t.end() << "s" << std::endl;

}
