#ifndef L3_PROJECTOR_H
#define L3_PROJECTOR_H

#include <cmath>
#include "PointCloud.h"
#include "Definitions.h"
#include "Datatypes.h"

namespace L3
{

template <typename T>
struct XYToXYZ
{
    
    XYToXYZ( L3::Pose* p ) : pose(p)
    {
        pt = Eigen::Matrix4f::Identity();
    }

    L3::Pose* pose;
    Eigen::Matrix4f pt;
    Eigen::Matrix4f result;

    Point<T> operator()( std::pair<T,T> t )
    {
        pt( 0, 3 ) = t.first;
        pt( 1, 3 ) = t.second;
        pt( 2, 3 ) = 0;

        result = pose->getHomogeneous() * pt;

        Point<T> point;

        point.x = result(0,3);
        point.x = result(1,3);
        point.z = result(2,3);

        return point;
    }
};


template <typename T>
struct RThetaToXY
{
    RThetaToXY( L3::LMS151* L )
    {
        counter   = L->num_scans;
        angle     = L->angle_start; 
        increment = L->angle_spacing;  
    }

    int counter;
    double angle, increment;
   
    T x,y;
    std::pair<T,T> operator()( T t )
    {
        x = t*cos( angle );
        y = t*sin( angle );
        angle += increment;
        return std::make_pair( x, y );
    }
};

template <typename T>
class Projector
{

    public:

        Projector()
        {
        }

        PointCloudXYZ<T> project( SWATHE* swathe )
        {
            SWATHE::iterator it = swathe->begin();
    
            L3::PointCloudXYZ<T> cloud;

            L3::Tools::Timer t;
            while( it != swathe->end() )
            {
                // Is there an associated scan?
                if (!(*it).second)
                {
                    it++;
                    continue;
                }
     
                // Project rtheta to xy
                L3::LMS151 L;
                L3::RThetaToXY<float> p( &L );
                std::vector< std::pair<double,double> > XY;
                std::transform(  (*it).second->ranges.begin(), (*it).second->ranges.end(), std::back_inserter( XY ), p );

                // Project xy to XYZ@Pose
                L3::XYToXYZ<double> p2( (*it).first ); 
                std::transform( XY.begin(), XY.end(), std::back_inserter( cloud.data ), p2 );

                it++;
            }
       
            return cloud;
        }
};

} // L3

#endif

