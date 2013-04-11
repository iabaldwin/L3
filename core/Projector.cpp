#include "Projector.h"

namespace L3
{

    template <typename T>
    void Projector<T>::project( SWATHE& swathe )
    {

#ifndef NDEBUG
        L3::Tools::Timer t;
        t.begin();
#endif
        int scan_counter, pair_counter, n = swathe.size();

        // Enough space?
        if ( n > allocated_size )
            reallocate( n );

        double x,y,range,angle;
        Eigen::Matrix4f* calib_ptr = &calibration;;

        // Swathe pointer
        SWATHE* swathe_ptr = &swathe;

        // Points pointer
        L3::Point<T>* points_ptr = cloud->points ;

#pragma omp parallel private(pair_counter,x,y,scan_counter ) shared(n, swathe_ptr, points_ptr, calib_ptr  )
        {
#pragma omp for  nowait
            //for( pair_counter=0; pair_counter < n; pair_counter+=20 ) 
            for( pair_counter=0; pair_counter < n; ++pair_counter ) 
            {
                Eigen::Matrix4f XY = Eigen::Matrix4f::Identity();
                
                for (scan_counter=0; scan_counter<541; scan_counter++) 
                {
                    // Compute angle 
                    double angle = scan_counter*(*swathe_ptr)[pair_counter].second->angle_spacing +  (*swathe_ptr)[pair_counter].second->angle_start; 

                    range = (*swathe_ptr)[pair_counter].second->ranges[scan_counter];  
                    //angle = scan_counter*.5+( -45 );
                    x = range*cos( angle );
                    y = range*sin( angle );

                    // 2D x&y
                    //x = (*swathe_ptr)[pair_counter].second->ranges[scan_counter] * cos( angle );
                    //y = (*swathe_ptr)[pair_counter].second->ranges[scan_counter] * sin( angle );

                    // Planar scan, no Z
                    XY(0,3) = x;
                    XY(1,3) = y;

                    // Project 3D point 
                    Eigen::Matrix4f res = ((*swathe_ptr)[pair_counter].first->getHomogeneous()*(*calib_ptr))*XY;

                    //points_ptr[ (pair_counter*541)+scan_counter ] = L3::Point<double>( res(0,3), res(1,3), res(2,3) );
                    points_ptr[(pair_counter*541)+scan_counter].x = res(0,3);
                    points_ptr[(pair_counter*541)+scan_counter].y = res(1,3);
                    points_ptr[(pair_counter*541)+scan_counter].z = res(2,3);
                }
            }
        }

        cloud->num_points = n*541;
    }

}

// Explicit Instantiation
template void L3::Projector<double>::project( std::vector< std::pair< boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> >, std::allocator<std::pair<boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> > > >&);

