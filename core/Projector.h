#pragma once

#include <cmath>

#include "Timing.h"
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

      Projector( L3::SE3* calib, L3::PointCloud<T>* cloud )
        : cloud(cloud),
        allocated_size(500),
        skip(1)
      {
        calibration = calib->getHomogeneous();

        // Pre-allocate
        cloud->points = new L3::Point<T>[ allocated_size*541 ];
      }

      L3::PointCloud<T>* cloud;
      size_t allocated_size;
      int skip;

      void reallocate( size_t size )
      {
        delete [] cloud->points;
        cloud->points = new L3::Point<T>[ size*541 ];
        allocated_size = size;
      }

      void project( SWATHE& swathe );

    };

  template <typename T>
    class ReflectanceProjector
    {
      Eigen::Matrix4f calibration;

      public:
      ReflectanceProjector( L3::SE3* calib, L3::PointCloudE<T>* cloud )
        : cloud(cloud),
        allocated_size(500),
        skip(1)
      {
        calibration = calib->getHomogeneous();
        // Pre-allocate
        cloud->points = new L3::PointE<T>[ allocated_size*541 ];
      }

      L3::PointCloudE<T>* cloud;
      size_t allocated_size;
      int skip;

      void reallocate( size_t size )
      {
        delete [] cloud->points;
        cloud->points = new L3::PointE<T>[ size*541 ];
        allocated_size = size;
      }

      void project( SWATHE& swathe );
    };
} // L3
