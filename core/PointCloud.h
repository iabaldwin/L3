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
    
};

template <typename T>
std::ostream& operator<<( std::ostream& o, const Point<T>& point )
{
    o << point.x << " " << point.y << " " << point.z;
    return o;
}


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

    /*
     *Data
     */
    L3::Point<T>* points;

    typedef L3::Point<T>* ITERATOR;

    ITERATOR begin()
    {
        return points;
    }

    ITERATOR end()
    {
        return (points+num_points);
    }

    size_t num_points;
      
};

template< typename T >
std::pair<T,T> mean( PointCloud<T>* cloud )
{
    typename L3::PointCloud<T>::ITERATOR it = cloud->begin();
  
    T x(0);
    T y(0);
    T z(0);

    while( it != cloud->end() )
    {
        // X 
        x += it->x;
        // Y 
        y += it->y;
        // Z 
        z += it->z;

        it++;
    }

    return std::make_pair(x/(T)cloud->num_points,y/(T)cloud->num_points);
}

template< typename T >
std::pair<T,T> min( PointCloud<T>* cloud )
{
    typename L3::PointCloud<T>::ITERATOR it = cloud->begin();
  
    T x = std::numeric_limits<T>::infinity();
    T y = std::numeric_limits<T>::infinity();
    T z = std::numeric_limits<T>::infinity();

    while( it != cloud->end() )
    {
        // X 
        x = std::min( x, it->x ); 
        // Y 
        y = std::min( y, it->y ); 
        // Z 
        z = std::min( z, it->z ); 

        it++;
    }

    return std::make_pair(x,y);
}

template< typename T >
std::pair<T,T> max( PointCloud<T>* cloud )
{
    typename L3::PointCloud<T>::ITERATOR it = cloud->begin();
 
    T x = -1* std::numeric_limits<T>::infinity();
    T y = -1* std::numeric_limits<T>::infinity();
    T z = -1* std::numeric_limits<T>::infinity();

    while( it != cloud->end() )
    {
        // X 
        x = std::max( x, it->x ); 
        // Y 
        y = std::max( y, it->y ); 
        // Z 
        z = std::max( z, it->z ); 

        it++;
    }

    return std::make_pair(x,y);
}




template <typename T>
PointCloud<T> samplePointCloud( PointCloud<T>* cloud, int size )
{
        PointCloud<T> sampled_cloud;

        // Generate random indices
        std::vector<int> random_indices( size );

        randomate r( cloud->num_points );

        std::generate( random_indices.begin(),  random_indices.end(), r );

        sampled_cloud.points = new L3::Point<T>[ size ];

        Point<T>* point_iterator = sampled_cloud.points;

        typename std::vector< int >::iterator index_iterator = random_indices.begin();

        while( index_iterator != random_indices.end() )
        {
            *point_iterator++ = cloud->points[ *index_iterator++ ];
        }

        sampled_cloud.num_points = size;

        return sampled_cloud;
}

template <typename T>
std::ostream& operator<<( std::ostream& o, PointCloud<T> cloud )
{
    for( size_t i=0; i<cloud.num_points; i++ )
        o << cloud.points[i] << std::endl;
    
    return o; 
}

//template <typename T>
//PointCloud<T>& operator>>( PointCloud<T>& cloud, L3::Utils::Locale l )
//{
    //typename PointCloud<T>::POINT_CLOUD_ITERATOR it = cloud.begin();

    //while( it != cloud.end() )
    //{
        //it->x -= l.x;
        //it->y -= l.y;
        //it->z -= l.z;
   
        //it++;
    //}

    //return cloud;
//}

//void transform( L3::Pose* t )
    //{
        //// 1. Compute mean
        //centroid c; 
        //c = std::for_each( data.begin(), data.end(), c );

        //// 2. Subtract
        //typename std::vector< Point<T> >::iterator it;
        //it = data.begin();
        
        //while( it != data.end() )
        //{
            //(*it).x -= c.x_mean;
            //(*it).y -= c.y_mean;
            //(*it).z -= c.z_mean;
            //it++;
        //}

        //// 3. Rotate
        //int point_counter, num_points = data.size();
        //std::vector< Point<T> >* tmp = &data;
        //T x,y,z;
        //Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();

//#pragma omp parallel private(point_counter, rot) shared(num_points, tmp, t )
        //{
            //Eigen::Matrix4f XYZ = Eigen::Matrix4f::Identity();
            
            //#pragma omp for  nowait
            //for (point_counter=0; point_counter<num_points; point_counter++) 
            //{
                //XYZ( 0, 3 ) = (*tmp)[point_counter].x;
                //XYZ( 1, 3 ) = (*tmp)[point_counter].y;
                //XYZ( 2, 3 ) = (*tmp)[point_counter].z;

                //rot = t->getHomogeneous()*XYZ;
           
                //(*tmp)[point_counter].x = rot( 0, 3 );
                //(*tmp)[point_counter].y = rot( 1, 3 );
                //(*tmp)[point_counter].z = rot( 2, 3 );
            //}
        //}

        //// 4. Translate
        ////it = data.begin();
        
        ////while( it != data.end() )
        ////{
            ////(*it).x += c.x_mean;
            ////(*it).y += c.y_mean;
            ////(*it).z += c.z_mean;
            ////it++;
        ////}
    //}

    
    //typedef typename std::vector< Point<T> >::iterator POINT_CLOUD_ITERATOR;

    //POINT_CLOUD_ITERATOR begin()
    //{
        //return data.begin();
    //}

    //POINT_CLOUD_ITERATOR end()
    //{
        //return data.end();
    //}


/*
 *Center
 */
template <typename T>
void centerPointCloud( PointCloud<T>* cloud )
{
    T x,y,z;

    for( size_t i=0; i<cloud->num_points; i++)
    {
        x += cloud->points[i].x;
        y += cloud->points[i].y;
        z += cloud->points[i].z;
    }

    x /= (double)cloud->num_points;
    y /= (double)cloud->num_points;
    z /= (double)cloud->num_points;

    std::cout << "Mean " << x << ":" << y << ":" << z << std::endl;
    
    for( size_t i=0; i<cloud->num_points; i++)
    {
        cloud->points[i].x -= x;
        cloud->points[i].y -= y;
        cloud->points[i].z -= z;
    }
}

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
