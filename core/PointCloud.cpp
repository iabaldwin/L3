#include "PointCloud.h"
#include "Datatypes.h"

namespace L3
{

/*
 *I/O
 */
template <typename T>
std::ostream& operator<<( std::ostream& o, const Point<T>& point )
{
    o << point.x << " " << point.y << " " << point.z;
    return o;
}

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
bool join( std::list< boost::shared_ptr<L3::PointCloud<T> > > clouds, boost::shared_ptr<L3::PointCloud<T> >& result )
{
    long unsigned int size = 0;

    L3::PointCloud<T>* resultant_cloud = new L3::PointCloud<T>();

    typename std::list< boost::shared_ptr<L3::PointCloud<T> > >::iterator it = clouds.begin();
   
    //Calculate size
    for( it = clouds.begin();
            it != clouds.end();
            it++ )
    {

        size += (*it)->num_points;
    }

    // Allocate
    L3::Point<T>* points = new L3::Point<T>[ size ];

    // Fill
    L3::Point<T>* ptr = points;

    for( it = clouds.begin();
            it != clouds.end();
            it++ )
    {
        std::copy( (*it)->points, (*it)->points+(*it)->num_points, ptr );
        ptr+= (*it)->num_points;
    }

    resultant_cloud->num_points = size;
    resultant_cloud->points     = points;

    result.reset( resultant_cloud );
    //return boost::shared_ptr<L3::PointCloud<T> >( resultant_cloud );
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

    std::cout << x << " " << y << " " <<  z << std::endl;

    for( size_t i=0; i<cloud->num_points; i++)
    {
        cloud->points[i].x -= x;
        cloud->points[i].y -= y;
        cloud->points[i].z -= z;
    }
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

/*
 *Manipulation
 */
template <typename T>
void transform( PointCloud<T>* cloud, L3::SE3* pose )
    {
        // 1. Compute mean
        std::pair<double,double> m = mean( cloud );

        // 2. Subtract
        typename PointCloud<T>::ITERATOR it = cloud->begin();
        
        while( it != cloud->end() )
        {
            (*it).x -= m.first;
            (*it).y -= m.second;
            it++;
        }

        // 3. Rotate
        int point_counter, num_points = cloud->num_points;
        T x,y,z;
        Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();

        Point<T>* tmp = cloud->begin();

#pragma omp parallel private(point_counter, rot) shared(num_points, tmp, t )
        {
            Eigen::Matrix4f XYZ = Eigen::Matrix4f::Identity();
            
            #pragma omp for  nowait
            for (point_counter=0; point_counter<num_points; point_counter++) 
            {
                XYZ( 0, 3 ) = tmp[point_counter].x;
                XYZ( 1, 3 ) = tmp[point_counter].y;
                XYZ( 2, 3 ) = tmp[point_counter].z;

                rot = pose->getHomogeneous()*XYZ;
           
                tmp[point_counter].x = rot( 0, 3 );
                tmp[point_counter].y = rot( 1, 3 );
                tmp[point_counter].z = rot( 2, 3 );
            }
        }

        // 4. Translate
        it = cloud->begin();
        
        while( it != cloud->end() )
        {
            (*it).x += m.first;
            (*it).y += m.second;
            it++;
        }
    }
}

// Explicit Instantiations
template void L3::transform( L3::PointCloud<double>* cloud, L3::SE3* p );

template std::pair<double, double>  L3::mean<double>(L3::PointCloud<double>*);
template std::pair<float, float>    L3::mean<float>(L3::PointCloud<float>*);
template bool L3::join( std::list< boost::shared_ptr<L3::PointCloud<double> > > clouds, boost::shared_ptr<L3::PointCloud<double> >& result );
template void L3::centerPointCloud<double>( PointCloud<double>* cloud );
template std::ostream& L3::operator<<( std::ostream& o, L3::PointCloud<double> cloud );

template L3::PointCloud<double>     L3::samplePointCloud( L3::PointCloud<double>* cloud, int size );
template std::pair<double,double>   L3::max( PointCloud<double>* cloud );
template std::pair<double,double>   L3::min( PointCloud<double>* cloud );

