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

        Projector()
        {
        }

        PointCloudXYZ<T> project( SWATHE* swathe )
        {
            L3::PointCloudXYZ<T> cloud;

#ifndef NDEBUG
            L3::Tools::Timer t;
            t.begin();
#endif
            int scan_counter, pair_counter, n, it, i;
            double x,y;

            //Project rtheta to xy
            //L3::LMS151 L;
            //L3::RThetaToXY<float> p( &L );

            std::vector< L3::Point<T> > points;
            points.resize( swathe->size() * 541 );

            std::cout << points.size() << " point" << std::endl;

            n=swathe->size();

            /* Fork a team of threads */
#pragma omp parallel private(pair_counter,x,y,scan_counter ) shared(n,swathe, points)
            {
                for( pair_counter=0; pair_counter< n; pair_counter++ ) 
                {
                    // Is there an associated scan?
                    if( (*swathe)[pair_counter].second )
                    {
                        Eigen::Matrix3f c = Eigen::Matrix3f::Identity();
            
                        Eigen::MatrixXf XY(3,541);
                        //Matrix541f XY(3,541);

                        #pragma omp for  nowait
                        for (scan_counter=0; scan_counter<541; scan_counter++) 
                        {
                            x = (*swathe)[pair_counter].second->ranges[scan_counter] * cos(0.0); 
                            y = (*swathe)[pair_counter].second->ranges[scan_counter] * sin(0.0); 
                            XY(0,scan_counter) = x;
                            XY(1,scan_counter) = y;
                            XY(2,scan_counter) = 0.0;
                        
                        } 

                        //Eigen::MatrixXf result = c*XY;
                          
                        //std::cout << c*XY << std::endl;

                        //int a = p( (*swathe)[it].second->ranges[i] );
                        //p( (*swathe)[it].second->ranges[i] );
                        //for( int i=0; i < (*it).second->ranges.size(); i++ )
                        //XY[i] = p( (*it).second->ranges[i] );
                        // Project xy to XYZ@Pose
                        //L3::XYToXYZ<double> p2( (*it).first ); 
                        //std::transform( XY.begin(), XY.end(), std::back_inserter( cloud.data ), p2 );

                    }
                }
            }

#ifndef NDEBUG
            std::cout << cloud.size() << " pts in " << t.end() << "s" << std::endl;
#endif
            return cloud;
        }
};

} // L3

#endif
