#ifndef L3_POINTCLOUD_H
#define L3_POINTCLOUD_H

#include <vector>

namespace L3
{

template< typename T>
struct Point
{

    T x,y,z;

    Point() 
    {
    }


    Point( T X, T Y, T Z ) : x(X), y(Y), z(Z)
    {
    }


};

template< typename T>
std::ostream& operator<<( std::ostream& o, const L3::Point<T>& p )
{
    o << p.x << "," << p.y << "," << p.z;
    return o;
}

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
struct PointCloudXYZ : PointCloud<T>
{

    PointCloudXYZ()
    {
    }

};

template <typename T>
std::ostream& operator<<( std::ostream& o, PointCloud<T> cloud)
{
        std::copy( cloud.data.begin(), 
                cloud.data.end(), 
                std::ostream_iterator< Point<T> >( o, "\n" ) );
        return o; 
}

} // L3

#endif

