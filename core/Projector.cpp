#include "Projector.h"

#include <fstream>

namespace L3
{
  template <typename T>
    struct point_predicate : std::unary_function< bool, L3::Point<T> > {
      bool operator()(L3::Point<T> t) {
        return ((t.x == 0)&& (t.y == 0) && (t.z == 0));
      }

    };

  template <typename T>
    void Projector<T>::project(SWATHE& swathe) {
#ifndef NDEBUG
      L3::Timing::SysTimer t;
      t.begin();
#endif
      int scan_counter;
      unsigned int n = swathe.size(), pair_counter;
      // Enough space?
      if (n > allocated_size) {
        reallocate(n);
      }

      double x,y,range,angle;
      Eigen::Matrix4f* calib_ptr = &calibration;;

      // Swathe pointer
      SWATHE* swathe_ptr = &swathe;

      // Points pointer
      L3::Point<T>* points_ptr = cloud->points ;

      int counter = 0;

      int local_skip = skip;

      {
        for(pair_counter=0; pair_counter < n; pair_counter += local_skip)  {
          Eigen::Matrix4f XY = Eigen::Matrix4f::Identity();

          for (scan_counter=0; scan_counter<541; scan_counter++)  {
            // Compute angle
            angle = scan_counter*(*swathe_ptr)[pair_counter].second->angle_spacing +  (*swathe_ptr)[pair_counter].second->angle_start;

            range = (*swathe_ptr)[pair_counter].second->ranges[scan_counter];

            if (range == 0) {
              continue;
            }

            //angle = scan_counter*.5+(-45);
            x = range*cos(angle);
            y = range*sin(angle);

            // Planar scan, no Z
            XY(0,3) = x;
            XY(1,3) = y;

            // Project 3D point
            Eigen::Matrix4f res = ((*swathe_ptr)[pair_counter].first->getHomogeneous()*(*calib_ptr))*XY;
            points_ptr[counter].x = res(0,3);
            points_ptr[counter].y = res(1,3);
            points_ptr[counter].z = res(2,3);
            counter++;
          }
        }
      }
      cloud->num_points = counter;
    }

  template <typename T>
    void ReflectanceProjector<T>::project(SWATHE& swathe) {
#ifndef NDEBUG
      L3::Timing::SysTimer t;
      t.begin();
#endif
      int scan_counter;
      unsigned int n = swathe.size(), pair_counter;
      // Enough space?
      if (n > allocated_size) {
        reallocate(n);
      }

      double x,y,range,angle;
      Eigen::Matrix4f* calib_ptr = &calibration;;

      // Swathe pointer
      SWATHE* swathe_ptr = &swathe;

      // Points pointer
      L3::PointE<T>* points_ptr = cloud->points ;

      int counter = 0;

      int local_skip = skip;

      for(pair_counter=0; pair_counter < n; pair_counter += local_skip) {
        Eigen::Matrix4f XY = Eigen::Matrix4f::Identity();

        for (scan_counter=0; scan_counter<541; scan_counter++) {
          // Compute angle
          angle = scan_counter*(*swathe_ptr)[pair_counter].second->angle_spacing +  (*swathe_ptr)[pair_counter].second->angle_start;

          range = (*swathe_ptr)[pair_counter].second->ranges[scan_counter];

          if (range == 0) {
            continue;
          }

          x = range*cos(angle);
          y = range*sin(angle);

          // Planar scan, no Z
          XY(0,3) = x;
          XY(1,3) = y;

          // Project 3D point
          Eigen::Matrix4f res = ((*swathe_ptr)[pair_counter].first->getHomogeneous()*(*calib_ptr))*XY;

          points_ptr[counter].x = res(0,3);
          points_ptr[counter].y = res(1,3);
          points_ptr[counter].z = res(2,3);
          points_ptr[counter].e = (*swathe_ptr)[pair_counter].second->reflectances[scan_counter];

          counter++;
        }
      }
      cloud->num_points = counter;
    }
} // L3

// Explicit Instantiation
template void L3::Projector<double>::project(std::vector<std::pair<boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> >, std::allocator<std::pair<boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> > > >&);
template void L3::ReflectanceProjector<double>::project(std::vector<std::pair<boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> >, std::allocator<std::pair<boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> > > >&);
