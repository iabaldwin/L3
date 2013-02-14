#ifndef L3_PROJECTOR_H
#define L3_PROJECTOR_H

#include <cmath>
#include <omp.h>

#include "PointCloud.h"
#include "Definitions.h"
#include "Datatypes.h"

typedef Eigen::Matrix<float, 3, 541> Matrix541f;

namespace L3
{

//template <typename T>
//struct XYToXYZ
//{
    
    //XYToXYZ( L3::Pose* p ) : pose(p)
    //{
        //pt = Eigen::Matrix4f::Identity();
    //}

    //L3::Pose* pose;
    //Eigen::Matrix4f pt;
    //Eigen::Matrix4f result;

    //Point<T> operator()( std::pair<T,T> t )
    //{
        //pt( 0, 3 ) = t.first;   // X
        //pt( 1, 3 ) = t.second;  // Y
        //pt( 2, 3 ) = 0;         // Z

        //result = pose->getHomogeneous() * pt;

        //Point<T> point;

        //point.x = result(0,3);
        //point.y = result(1,3);
        //point.z = result(2,3);

        //return point;
    //}
//};


//template <typename T>
//struct RThetaToXY
//{
    //RThetaToXY( L3::LMS151* L )
    //{
        //counter   = L->num_scans;
        //angle     = L->angle_start; 
        //increment = L->angle_spacing;  
    //}

    //int counter;
    //double angle, increment;
   
    //T x,y;
    //std::pair<T,T> operator()( T t )
    //{
        //x = t*cos( angle );
        //y = t*sin( angle );
        //angle += increment;
        //return std::make_pair( x, y );
    //}
//};

template <typename T>
class Projector
{

    public:

        PointCloudXYZ<T> project( SWATHE* swathe )
        {
            L3::PointCloudXYZ<T> cloud;

#ifndef NDEBUG
            L3::Tools::Timer t;
            t.begin();
#endif
            int scan_counter, pair_counter, n; 
            double x,y;

            std::vector< L3::Point<T> > points;
            points.resize( swathe->size() * 541 );

            n=swathe->size();

            Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();

            /* Fork a team of threads */
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

            cloud.data.assign( points.begin(), points.end() );
#ifndef NDEBUG
            std::cout << cloud.size() << " pts in " << t.end() << "s" << std::endl;
#endif
            return cloud;
        }
};

} // L3

#endif
