#ifndef L3_PROJECTOR_H
#define L3_PROJECTOR_H

#include <cmath>
#include <omp.h>

#include "PointCloud.h"
#include "Definitions.h"
#include "Datatypes.h"

namespace L3
{

template <typename T>
class Projector
{
    Eigen::Matrix4f calibration;

    public:
    
        L3::PointCloud<T>* cloud;

        Projector( L3::SE3* calib, L3::PointCloud<T>* CLOUD ) : cloud(CLOUD), allocated_size(500)
        {
            calibration = calib->getHomogeneous(); 

            // Pre-allocate
            cloud->points = new L3::Point<T>[ allocated_size*541 ]; 
        }

        size_t allocated_size;

        ~Projector()
        {
            delete [] cloud->points;
        }

        void reallocate( size_t size )
        {
            delete [] cloud->points;
            cloud->points = new L3::Point<T>[ size*541 ]; 
            allocated_size = size;
        }

        /*
         *Do projection for each point
         */

        void project( SWATHE& swathe )
        {

#ifndef NDEBUG
            L3::Tools::Timer t;
            t.begin();
#endif
            unsigned int scan_counter, pair_counter;
            unsigned int n = swathe.size();
          
            if ( n > allocated_size )
                reallocate( n );
            
            double x,y;

            Eigen::Matrix4f tmp         = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f _calib      = this->calibration;
            Eigen::Matrix4f* calib_ptr  = &_calib;

            // Swathe reference
            SWATHE* swathe_ptr = &swathe;

            // Points reference
            L3::Point<T>* points_ptr = cloud->points ;

//#pragma omp parallel private(pair_counter,x,y,scan_counter,tmp ) shared(n, swathe_ptr, points_ptr, calib_ptr  )
            {
                for( pair_counter=0; pair_counter < n; pair_counter+=1 ) 
                {
                    Eigen::Matrix4f XY = Eigen::Matrix4f::Identity();

//#pragma omp for  nowait
                    for (scan_counter=0; scan_counter<541; scan_counter++) 
                    {

                        double angle = scan_counter*(*swathe_ptr)[pair_counter].second->angle_spacing +  (*swathe_ptr)[pair_counter].second->angle_start; 

                        x = (*swathe_ptr)[pair_counter].second->ranges[scan_counter] * cos( angle );
                        y = (*swathe_ptr)[pair_counter].second->ranges[scan_counter] * sin( angle );

                        // Planar scan, no Z
                        XY(0,3) = x;
                        XY(1,3) = y;
                        XY(2,3) = 0.0;

                        tmp = ((*swathe_ptr)[pair_counter].first->getHomogeneous()*(*calib_ptr))*XY;

                        //points[(pair_counter*541)+scan_counter] = L3::Point<T>( tmp(0,3), tmp(1,3), tmp(2,3) );
                        points_ptr[(pair_counter*541)+scan_counter].x = tmp(0,3);
                        points_ptr[(pair_counter*541)+scan_counter].y = tmp(1,3);
                        points_ptr[(pair_counter*541)+scan_counter].z = tmp(2,3);
                    }
                }
            }
        
            cloud->num_points = n*541;
        }
    };

} // L3

#endif
