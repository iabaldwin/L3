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
    boost::shared_ptr<L3::PointCloud<double> > cloud( new L3::PointCloud<double>() );

    std::vector< L3::Point<double> > randoms(2*100000);
    std::generate( randoms.begin(), randoms.end(), randomate<double> );

    cloud->num_points = randoms.size();
    cloud->points = new L3::Point<double>[ cloud->num_points ];
    std::copy( randoms.begin(), randoms.end(), cloud->points );

    boost::shared_ptr<L3::PointCloud<double> > copy( new L3::PointCloud<double>() );

    std::string result;
    result = L3::copy( cloud.get(), copy.get() ) ? "Success" : "Failure";
    std::cout << result << std::endl;
}
