#ifndef L3_POINTCLOUD_H
#define L3_POINTCLOUD_H

#include <vector>

namespace L3
{

template< typename T>
struct Point
{

    T x,y,z;

    Point( T X, T Y, T Z ) : x(X), y(Y), z(Z)
    {
    }


};

template< typename T>
struct PointCloud
{
    typedef typename std::vector< Point<T> >::iterator Iterator;
    
    size_t size()
    {
        return data.size();
    }

    // Data
    std::vector< Point<T> > data;

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
        output << std::endl; 
    }

};

template <typename T>
struct PointCloudXYZ : PointCloud<T>
{

    PointCloudXYZ()
    {
    }
    
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

