#ifndef L3_POINTCLOUD_H
#define L3_POINTCLOUD_H

namespace L3
{

struct PointCloud
{

};

template <typename T>
struct Writer
{

    Writer( std::ostream& o ) : output(o)
    {

    }

    std::ostream& output;

    void operator()( std::vector<T> t )
    {
        std::copy( t.begin(), t.end(), std::ostream_iterator<T>( output, " " ) );
    }
};

template <typename T>
struct PointCloudXYZ : PointCloud
{

    PointCloudXYZ()
    {

    }

    // Data
    std::vector< std::vector<double> > data;

};

template <typename T>
std::ostream& operator<<( std::ostream& o, PointCloudXYZ<T> cloud)
{
        Writer<T> w( std::cout );
        std::for_each( cloud.data.begin(), cloud.data.end(), w );
        return o; 
}

} // L3


#endif

