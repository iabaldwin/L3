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

}
