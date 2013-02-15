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

        PointCloudXYZ<T> project( SWATHE* swathe )
        {
#ifndef NDEBUG
            L3::Tools::Timer t;
            t.begin();
#endif

            L3::PointCloudXYZ<T> cloud;

            int scan_counter, pair_counter, n; 
            double x,y;
            n=swathe->size();
            
            std::vector< L3::Point<T> > points;
            points.resize( n * 541 );

            Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();

#pragma omp parallel private(pair_counter,x,y,scan_counter,tmp ) shared(n, swathe, points)
            {
                for( pair_counter=0; pair_counter< n; pair_counter++ ) 
                {
                    // Is there an associated scan?
                    if( (*swathe)[pair_counter].second )
                    {
                        Eigen::Matrix4f XY = Eigen::Matrix4f::Identity();
                        Eigen::Matrix4f c = Eigen::Matrix4f::Identity();

                        #pragma omp for  nowait
                        for (scan_counter=0; scan_counter<541; scan_counter++) 
                        {
                            x = (*swathe)[pair_counter].second->ranges[scan_counter] * cos(0.0); 
                            y = (*swathe)[pair_counter].second->ranges[scan_counter] * sin(0.0); 
                    
                            // Planar scan, no Z
                            XY(0,3) = x;
                            XY(1,3) = y;
                            XY(2,3) = 0.0;

                            // Project @ Pose
                            tmp = (*swathe)[pair_counter].first->getHomogeneous()* XY;
                                
                            points[pair_counter*541+scan_counter] = L3::Point<T>( tmp(0,3), tmp(1,3), tmp(2,3) );

                        }
                    }
                }
            }

            // Copy
            cloud.data.assign( points.begin(), points.end() );
#ifndef NDEBUG
            std::cout << cloud.size() << " pts in " << t.end() << "s" << std::endl;
#endif
            return cloud;
        }
};

} // L3

#endif
