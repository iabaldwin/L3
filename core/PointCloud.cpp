#include "PointCloud.h"
#include "Datatypes.h"

namespace L3
{

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
boost::shared_ptr<L3::PointCloud<T> > join( std::list< boost::shared_ptr<L3::PointCloud<T> > > clouds )
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

    return boost::shared_ptr<L3::PointCloud<T> >( resultant_cloud );
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

}

// Explicit Instantiations
template std::pair<double, double> L3::mean<double>(L3::PointCloud<double>*);
template std::pair<float, float> L3::mean<float>(L3::PointCloud<float>*);
template boost::shared_ptr<L3::PointCloud<double> > L3::join( std::list< boost::shared_ptr<L3::PointCloud<double> > > clouds );


