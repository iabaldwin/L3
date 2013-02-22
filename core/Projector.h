#ifndef L3_PROJECTOR_H
#define L3_PROJECTOR_H

#include <cmath>
#include <omp.h>

#include "PointCloud.h"
#include "Definitions.h"
#include "Datatypes.h"

//#define THRESHOLD 0.4
#define THRESHOLD 0.0

namespace L3
{

template <typename T>
class Projector
{

    public:

        Projector()
        {
            calibration = Eigen::Matrix4f::Identity();            
        }

        Projector( L3::SE3* calib )
        {
            calibration = calib->getHomogeneous(); 
        }

        Eigen::Matrix4f calibration;

        PointCloudXYZ<T> project( SWATHE& swathe )
        {

#ifndef NDEBUG
            L3::Tools::Timer t;
            t.begin();
#endif
            L3::PointCloudXYZ<T> cloud;
            //std::vector< L3::Point<T> > points;
            //points.resize( n * 541 );


            
            int scan_counter, pair_counter, n = swathe.size();
            double x,y;
            
            L3::Point<T>* points = new L3::Point<T>[ n*541 ]; 
       
            Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f calib = this->calibration;

            // Swathe reference
            SWATHE* swathe_ptr = &swathe;

            int skip = 10;

#pragma omp parallel private(pair_counter,x,y,scan_counter,tmp,skip ) shared(n, swathe_ptr, points, calib )
            {
                //for( pair_counter=0; pair_counter < n; pair_counter++ ) 
                //for( pair_counter=0; pair_counter < n; pair_counter += 10 ) 
                for( pair_counter=0; pair_counter < n; pair_counter++ ) 
                {

                    Eigen::Matrix4f XY = Eigen::Matrix4f::Identity();
                    Eigen::Matrix4f c  = Eigen::Matrix4f::Identity();

#pragma omp for  nowait
                    for (scan_counter=0; scan_counter<541; scan_counter++) 
                    {
                        //if ( (*swathe_ptr)[pair_counter].second->ranges[scan_counter] > THRESHOLD )
                        //{
                        x = (*swathe_ptr)[pair_counter].second->ranges[scan_counter] * cos( scan_counter*(*swathe_ptr)[pair_counter].second->angle_spacing +  (*swathe_ptr)[pair_counter].second->angle_start); 
                        y = (*swathe_ptr)[pair_counter].second->ranges[scan_counter] * sin( scan_counter*(*swathe_ptr)[pair_counter].second->angle_spacing +  (*swathe_ptr)[pair_counter].second->angle_start); 

                        //std::cout << x << ":" << y << std::endl;

                        // Planar scan, no Z
                        XY(0,3) = x;
                        XY(1,3) = y;
                        XY(2,3) = 0.0;

                        // Project @ Pose
                        tmp = (*swathe_ptr)[pair_counter].first->getHomogeneous()* XY;
                        //tmp = calib* (*swathe_ptr)[pair_counter].first->getHomogeneous()* XY;
                            points[(pair_counter*541)+scan_counter] = L3::Point<T>( tmp(0,3), tmp(1,3), tmp(2,3) );

                        //}
                        //else
                            //points[(pair_counter*541)+scan_counter] = L3::Point<T>( 0, 0, 0 );

                    }
                }
            }

            // Copy
            //cloud.data.assign( points.begin(), points.end() );
            cloud.data.assign( points, points+(n*541));
#ifndef NDEBUG
            std::cout << cloud.size() << " pts in " << t.end() << "s" << std::endl;
#endif
            return cloud;
        }
};

} // L3

#endif
