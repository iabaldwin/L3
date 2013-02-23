#ifndef L3_POINTCLOUD_H
#define L3_POINTCLOUD_H

#include <vector>
#include <gsl/gsl_histogram2d.h>

#include <omp.h>

#include "Utils.h"
#include "Datatypes.h"

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
    
    virtual void print( std::ostream& o ) const
    {
        o << x << " " << y << " " << z;
    }
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

    void print( std::ostream& o ) const
    {
        o << x << " " << y << " " << z << " " << r << " " << g << " " << b; 
    }

};


template< typename T>
std::ostream& operator<<( std::ostream& o, const L3::Point< T> & p )
{
    p.print( o ); 
    return o;
}

/*
 *Cloud types
 */
template< typename T>
struct PointCloud 
{
    typedef typename std::vector< Point< T > >::iterator Iterator;
    
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
        int point_counter, num_points = data.size();
        std::vector< Point<T> >* tmp = &data;
        T x,y,z;
        Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();

#pragma omp parallel private(point_counter, rot) shared(num_points, tmp, t )
        {
            Eigen::Matrix4d XYZ = Eigen::Matrix4d::Identity();
            
            #pragma omp for  nowait
            for (point_counter=0; point_counter<num_points; point_counter++) 
            {
                XYZ( 0, 3 ) = (*tmp)[point_counter].x;
                XYZ( 1, 3 ) = (*tmp)[point_counter].y;
                XYZ( 2, 3 ) = (*tmp)[point_counter].z;

                rot = t->getHomogeneous()*XYZ;
           
                (*tmp)[point_counter].x = rot( 0, 3 );
                (*tmp)[point_counter].y = rot( 1, 3 );
                (*tmp)[point_counter].z = rot( 2, 3 );
            }
        }

        // 4. Translate
        //it = data.begin();
        
        //while( it != data.end() )
        //{
            //(*it).x += c.x_mean;
            //(*it).y += c.y_mean;
            //(*it).z += c.z_mean;
            //it++;
        //}
    }

    /*
     *Data
     */
    std::vector< Point<T> > data;

    typedef typename std::vector< Point<T> >::iterator POINT_CLOUD_ITERATOR;

    POINT_CLOUD_ITERATOR begin()
    {
        return data.begin();
    }

    POINT_CLOUD_ITERATOR end()
    {
        return data.end();
    }

    /*
     *Statistics
     */
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
PointCloud<T> samplePointCloud( PointCloud<T>& cloud, int size )
{
        PointCloud<T> sampled_cloud;

        // Generate random indices
        std::vector<int> random_indices( size );

        randomate r( cloud.size() ) ;

        std::generate( random_indices.begin(),  random_indices.end(), r );

        sampled_cloud.data.resize( size );

        typename std::vector< Point<T> >::iterator point_iterator = sampled_cloud.data.begin();
        typename std::vector< int >::iterator index_iterator = random_indices.begin();

        while( index_iterator != random_indices.end() )
        {
            *point_iterator++ = cloud.data[ *index_iterator++ ];
        }

        return sampled_cloud;
}

template <typename T>
std::ostream& operator<<( std::ostream& o, PointCloud<T> cloud )
{
    std::copy( cloud.data.begin(), 
            cloud.data.end(), 
            std::ostream_iterator< Point<T> >( o, "\n" ) );


    return o; 
}

template <typename T>
PointCloud<T>& operator>>( PointCloud<T>& cloud, L3::Utils::Locale l )
{
    typename PointCloud<T>::POINT_CLOUD_ITERATOR it = cloud.begin();

    while( it != cloud.end() )
    {
        it->x -= l.x;
        it->y -= l.y;
        it->z -= l.z;
   
        it++;
    }

    return cloud;
}

/*
 *Center
 */
template <typename T>
void centerPointCloud( PointCloud<T>& cloud )
{
    T x,y,z;

    int counter = 0;

    typename PointCloud<T>::POINT_CLOUD_ITERATOR it = cloud.begin();
    
    while( it != cloud.end() )
    {
        x += it->x;
        y += it->y;
        z += it->z;
        counter++; 
        it++;
    }

    x /= (double)counter;
    y /= (double)counter;
    z /= (double)counter;

    std::cout << "Mean " << x << ":" << y << ":" << z << std::endl;

    it = cloud.begin();
    while( it != cloud.end() )
    {
        it->x -= x;
        it->y -= y;
        it->z -= z;
        it++;
    }
}




} // L3

#endif
