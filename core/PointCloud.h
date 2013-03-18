#ifndef L3_POINTCLOUD_H
#define L3_POINTCLOUD_H

#include <vector>
#include <gsl/gsl_histogram2d.h>

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

template <typename T>
std::ostream& operator<<( std::ostream& o, const Point<T>& point );

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

template <typename T>
std::ostream& operator<<( std::ostream& o, const PointRGB<T>& point );


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

/*
* Statistics
*/
template <typename T>
struct histogram
{
    float delta;
    unsigned int num_bins;
    
    histogram(  float x_centre=0.f, 
                float x_lower=50.0f, 
                float x_upper=50.0f, 
                float y_centre=0.f, 
                float y_lower=50.0f, 
                float y_upper=50.0f, 
                unsigned int bins=1000 ) : num_bins(bins)
    {
        hist =  gsl_histogram2d_alloc (num_bins, num_bins);

        gsl_histogram2d_set_ranges_uniform (hist, 
                                            x_centre-x_lower, 
                                            x_centre+x_upper, 
                                            y_centre-y_lower, 
                                            y_centre+y_upper );
   
        delta = 100.0/num_bins;
    }

    ~histogram()
    {
        gsl_histogram2d_free( hist );
    }


    void reset()
    {
        gsl_histogram2d_reset( hist );
    }

    unsigned int bin( size_t x, size_t y  )
    {
        return gsl_histogram2d_get(hist,x,y); 
    }

    std::pair<float, float> coords( size_t x, size_t y  )
    {
        return std::make_pair( hist->xrange[x], hist->yrange[y] );
    }

    gsl_histogram2d* hist;

    void operator()( L3::PointCloud<T>* cloud )
    {
        for( typename L3::PointCloud<T>::ITERATOR it = cloud->begin(); it != cloud->end(); it++ )
            gsl_histogram2d_increment( hist, it->x, it->y );
    }

};

} // L3

#endif
