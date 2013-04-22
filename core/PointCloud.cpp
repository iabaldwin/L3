#include "PointCloud.h"
#include "Datatypes.h"

#include <boost/random.hpp>

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
        }

    template <typename T>
        bool sample( PointCloud<T>* input,  PointCloud<T>* output, int size )
        {
            // Generate random indices
            std::vector<int> random_indices( size );
            randomate r( input->num_points );
            std::generate( random_indices.begin(),  random_indices.end(), r );

            output->num_points = size;
            output->points = new L3::Point<T>[ output->num_points ];

            typename std::vector< int >::iterator index_iterator = random_indices.begin();

            Point<T>* point_iterator = output->points;

            while( index_iterator != random_indices.end() )
            {
                *point_iterator++ = input->points[ *index_iterator++ ];
            }


            return ( std::distance( output->begin(), output->end() ) == output->num_points );
        }

    template <typename T>
        std::ostream& operator<<( std::ostream& o, const PointCloud<T>& cloud )
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

    /*
     *Manipulation
     */
    template <typename T>
        void transform( PointCloud<T>* cloud, L3::SE3 * pose )
        {
            typename PointCloud<T>::ITERATOR it = cloud->begin();

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

            
        }

    /*
     *  Manipulation
     */
    template <typename T>
        void translate( PointCloud<T>* cloud, L3::SE3 const * pose )
        {
            // 2. Subtract
            typename PointCloud<T>::ITERATOR it = cloud->begin();

            while( it != cloud->end() )
            {
                (*it).x += pose->X();
                (*it).y += pose->Y();
                it++;
            }
        }

    /*
     *  Utilities
     */
    template <typename T>
        bool copy( PointCloud<T>* src, PointCloud<T>* dest )
        {
            dest->num_points = src->num_points;
            dest->points = new Point<T>[ dest->num_points ];
            std::copy( src->begin(), src->end(), dest->begin() );
            return ( std::distance( dest->begin(), dest->end() ) == dest->num_points );
        }

    template <typename T>
        void gaussianCloud( PointCloud<T>* cloud, double variance )
        {
            // Generator
            boost::mt19937 rng;
            boost::normal_distribution<> normal(0.0, variance );
            boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, normal );

            L3::PointCloud<double>::ITERATOR it = cloud->begin();
      
            while ( it != cloud->end() )
            {
                *it++ = L3::Point<double>( var_nor(), var_nor(), var_nor() );
            }

        }

    template <typename T>
        void allocate( PointCloud<T>* cloud, size_t size  )
        {
            assert( !cloud->points );
            assert( cloud->num_points == 0 );

            cloud->points = new L3::Point<T>[ size ];
            cloud->num_points = size;
        }


}


/*
 *Explicit Instantiations
 */
        
template void                           L3::allocate( PointCloud<double>* cloud, size_t size  );
template void                           L3::transform( L3::PointCloud<double>*, L3::SE3* );
template void                           L3::translate( L3::PointCloud<double>*, L3::SE3 const* );
template bool                           L3::copy( PointCloud<double>*, PointCloud<double>*);

template std::pair<double, double>      L3::mean<double>(L3::PointCloud<double>*);
template std::pair<float, float>        L3::mean<float>(L3::PointCloud<float>*);
template bool L3::join( std::list< boost::shared_ptr<L3::PointCloud<double> > > clouds, boost::shared_ptr<L3::PointCloud<double> >& result );
template void                           L3::centerPointCloud<double>( PointCloud<double>* cloud );

template bool                           L3::sample( L3::PointCloud<double>*,  L3::PointCloud<double>*, int );
template std::pair<double,double>       L3::max( PointCloud<double>* );
template std::pair<double,double>       L3::min( PointCloud<double>* );

template void                           L3::gaussianCloud( PointCloud<double>* cloud, double variance );

template std::ostream& L3::operator<<( std::ostream& o, const L3::PointCloud<double>& cloud );
