#ifndef L3_POINTCLOUD_H
#define L3_POINTCLOUD_H

#include <vector>
#include <gsl/gsl_histogram2d.h>

#include <omp.h>

#include "Datatypes.h"

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

    struct centroid 
    {
        int counter;
        T x_mean, y_mean, z_mean; 
        centroid() :counter(0), x_mean(0), y_mean(0), z_mean(0)
        {
            
        }

        void operator()( Point<T> t )
        {
            counter++;
            
            x_mean += t.x;
            y_mean += t.y;
            z_mean += t.z;
   
            x_mean /= counter;
            y_mean /= counter;
            z_mean /= counter;
        }
    };

    void transform( L3::Pose* t )
    {
        // 1. Compute mean
        centroid c; 
        c = std::for_each( data.begin(), data.end(), c );

        // 2. Subtract
        typename std::vector< Point<T> >::iterator it;
        it = data.begin();
        
        while( it != data.end() )
        {
            (*it).x -= c.x_mean;
            (*it).y -= c.y_mean;
            (*it).z -= c.z_mean;
            it++;
        }

        // 3. Rotate
        std::cout << t->getHomogeneous() << std::endl;      

        // 4. Translate
    
    }

    // Data
    std::vector< Point<T> > data;

    struct histogrammer
    {
        histogrammer( gsl_histogram2d* HISTOGRAM ) : histogram(HISTOGRAM)
        {
        }

        gsl_histogram2d* histogram;

        void operator()( Point<T> t )
        {
            gsl_histogram2d_increment( histogram, t.x, t.y );
        }

    };

    void histogram()
    {
         gsl_histogram2d* hist =  gsl_histogram2d_alloc (1000, 1000);

        gsl_histogram2d_set_ranges_uniform (hist, 0.0, 100.0, 0.0, 100.0);

        histogrammer h( hist );

        std::for_each( data.begin(), data.end(), h );
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
std::ostream& operator<<( std::ostream& o, PointCloud<T> cloud)
{
        std::copy( cloud.data.begin(), 
                cloud.data.end(), 
                std::ostream_iterator< Point<T> >( o, "\n" ) );
        return o; 
}

} // L3

#endif
