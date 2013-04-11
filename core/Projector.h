#ifndef L3_PROJECTOR_H
#define L3_PROJECTOR_H

#include <cmath>
#include <omp.h>

#include "Tools.h"
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
        size_t allocated_size;

        Projector( L3::SE3* calib, L3::PointCloud<T>* cloud ) : cloud(cloud), allocated_size(500)
        {
            calibration = calib->getHomogeneous(); 

            // Pre-allocate
            cloud->points = new L3::Point<T>[ allocated_size*541 ]; 
        }

        void reallocate( size_t size )
        {
            delete [] cloud->points;
            cloud->points = new L3::Point<T>[ size*541 ]; 
            allocated_size = size;
        }

        void project( SWATHE& swathe );
        
    };

} // L3

#endif
