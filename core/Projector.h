#ifndef L3_PROJECTOR_H
#define L3_PROJECTOR_H

#include "PointCloud.h"
#include "Definitions.h"

namespace L3
{

template <typename T>
struct RThetaToXY
{
    void operator()( std::pair<T, T> rtheta )
    {

    }
};


class Projector
{

    public:

        Projector()
        {


        }

        PointCloud project( const POSE_SEQUENCE& poses, const LIDAR_SEQUENCE& lidar )
        {


        }

        PointCloud project( SWATHE* swathe )
        {
            SWATHE::iterator it = swathe->begin();
       
            while( it != swathe->end() )
            {
                if (!(*it).second)
                {
                    it++;
                    continue;
                }
       
                // Project rtheta to xy
                L3::RThetaToXY<float> p;

                //std::transform(  (*it).second->ranges.begin(), (*it).second->ranges.end(), 

                it++;
            }
        }
};

} // L3

#endif

