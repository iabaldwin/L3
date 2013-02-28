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

        PointCloud<T> project( SWATHE& swathe )
        {

#ifndef NDEBUG
            L3::Tools::Timer t;
            t.begin();
#endif
            L3::PointCloud<T> cloud;
            
            int scan_counter, pair_counter;
            int n = swathe.size();
            double x,y;
            
            L3::Point<T>* points = new L3::Point<T>[ n*541 ]; 
       
            Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f _calib = this->calibration;
            Eigen::Matrix4f* calib_ptr = &_calib;

            // Swathe reference
            SWATHE* swathe_ptr = &swathe;

#pragma omp parallel private(pair_counter,x,y,scan_counter,tmp ) shared(n, swathe_ptr, points, calib_ptr  )
            {
                for( pair_counter=0; pair_counter < n; pair_counter+=2 ) 
                {
                    Eigen::Matrix4f XY = Eigen::Matrix4f::Identity();

#pragma omp for  nowait
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
                        //tmp = ((*swathe_ptr)[pair_counter].first->getHomogeneous())*XY;
                        points[(pair_counter*541)+scan_counter] = L3::Point<T>( tmp(0,3), tmp(1,3), tmp(2,3) );
                    }

                }
            }

            // Copy
            cloud.data.assign( points, points+(n*541));
            delete [] points;

#ifndef NDEBUG
            std::cout << cloud.size() << " pts in " << t.end() << "s" << std::endl;
#endif
            return cloud;
        }
};

} // L3

#endif
