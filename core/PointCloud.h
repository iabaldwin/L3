#ifndef L3_POINTCLOUD_H
#define L3_POINTCLOUD_H

#include <vector>
#include <omp.h>

#include "Utils.h"

/*
 *Helpers
 */
struct randomate 
{ 
    randomate( int MODULO )  : modulo(MODULO)
    {}

    int modulo;

    int operator()( )
    {
        return rand()  % modulo;
    }
};

/*
 *L3
 */
namespace L3
{

/*
 *Point types
 */
template< typename T>
struct Point
{
    Point() : x(0), y(0), z(0)
    {
    }

    Point( T X, T Y, T Z ) : x(X), y(Y), z(Z)
    {
    }
    
    T x,y,z;
    
};

template< typename T>
struct PointRGB : Point<T>
{

    T x,y,z;
    T r,g,b;

    PointRGB() : Point<T>(0,0,0),  r(0), g(0), b(0)
    {
    }


    PointRGB( T X, T Y, T Z, T R, T G, T B ) : Point<T>(x,y,z), r(R), g(G), b(B)
    {
    }

};


/*
 *Cloud types
 */
template< typename T>
struct PointCloud 
{

    PointCloud() : num_points(0), points(NULL)
    {
    }

    size_t num_points;
    L3::Point<T>* points;


    ~PointCloud()
    {
        if (num_points > 0 && points )
            delete [] points;
    }

    typedef L3::Point<T>* ITERATOR;

    ITERATOR begin()
    {
        return points;
    }

    ITERATOR end()
    {
        return (points+num_points);
    }
      
};

template< typename T >
std::pair<T,T> mean( PointCloud<T>* cloud );

template< typename T >
std::pair<T,T> min( PointCloud<T>* cloud );

template< typename T >
std::pair<T,T> max( PointCloud<T>* cloud );

template <typename T>
boost::shared_ptr<L3::PointCloud<T> > join( std::list< boost::shared_ptr<L3::PointCloud<T> > > clouds );

template <typename T>
PointCloud<T> samplePointCloud( PointCloud<T>* cloud, int size );

template <typename T>
std::ostream& operator<<( std::ostream& o, PointCloud<T> cloud );

template <typename T>
void centerPointCloud( PointCloud<T>* cloud );

template <typename T>
void transform( PointCloud<T>* cloud, L3::SE3* pose );

template <typename T>
std::ostream& operator<<( std::ostream& o, const Point<T>& point );

template <typename T>
std::ostream& operator<<( std::ostream& o, const PointRGB<T>& point );

} // L3

#endif
