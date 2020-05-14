#include "ScanMatching.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt_2d.h>
#include <pcl/registration/gicp.h>

#include <pcl/features/normal_3d.h>

#define TRAJECTORY_LIMIT 500

int do_projection(const L3::LMS151& current_scan, double* matrix, double threshold = 5.0) {
  double* matrix_ptr = matrix;

  double range,angle,x,y,z=0.0;

  int counter = 0;

  for (int scan_counter=0; scan_counter<541; scan_counter++)  {
    // Compute angle
    angle = scan_counter*current_scan.angle_spacing +  current_scan.angle_start;
    range = current_scan.ranges[scan_counter];

    if (range < threshold || range > 20.0) {
      continue;
    }

    x = range*cos(angle);
    y = range*sin(angle);

    *matrix_ptr++ = x;
    *matrix_ptr++ = y;
    *matrix_ptr++ = z;

    counter++;
  }

  return counter;
}

namespace L3 {
  namespace ScanMatching {

    bool ICP::match( const std::pair< double,  boost::shared_ptr< L3::LMS151 > > current_scan, Eigen::Matrix4f& transformation)  {
      if (!initialised) {
        scan.reset(new double[541*3]);
        scan_points = do_projection(*(current_scan.second), scan.get());
        initialised = true;
        return false;
      }

      /*
       *  Scan
       */
      cloud_in->width    = scan_points;
      cloud_in->height   = 1;
      cloud_in->is_dense = false;

      cloud_in->points.resize (cloud_in->width * cloud_in->height);

      double* cloud_ptr = scan.get();

      for(int i=0; i< scan_points; i++) {
        cloud_in->points[i].x = *cloud_ptr++;
        cloud_in->points[i].y = *cloud_ptr++;
        cloud_in->points[i].z = *cloud_ptr++;
      }

      /*
       *  Putative
       */
      putative.reset(new double[541*3]);
      putative_points  = do_projection((*current_scan.second), putative.get());

      cloud_out->width    = putative_points;
      cloud_out->height   = 1;
      cloud_out->is_dense = false;

      cloud_out->points.resize (cloud_out->width * cloud_out->height);

      cloud_ptr = putative.get();

      for(int i=0; i< putative_points; i++) {
        cloud_out->points[i].x = *cloud_ptr++;
        cloud_out->points[i].y = *cloud_ptr++;
        cloud_out->points[i].z = *cloud_ptr++;
      }

      pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);

      ne.setKSearch (30);

      ne.setInputCloud (cloud_out);
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_out(new pcl::PointCloud<pcl::PointNormal>);

      // Compute the features
      ne.compute (*cloud_normals_out);

      ne.setInputCloud (cloud_in);
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_in(new pcl::PointCloud<pcl::PointNormal>);

      // Compute the features
      ne.compute (*cloud_normals_in);

      pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;

      registration.setMaximumIterations (50);

      registration.setInputSource(cloud_out);
      registration.setInputTarget(cloud_in);

      // Alignment
      registration.align(*final);

      transformation = registration.getFinalTransformation();

      // Do swap
      scan = putative;
      scan_points = putative_points;

      return registration.hasConverged();
    }

    bool Engine::update(double time) {
      if(this->windower->window.empty()) {
        return false;
      }

      // Get the most recent
      std::pair< double, boost::shared_ptr< L3::LMS151 > > scan = this->windower->window.back();

      Eigen::Matrix4f transformation;

      L3::WriteLock write_lock(this->mutex);

      if(matcher->match(scan, transformation))  {
        // Compute delta
        current_transformation *= transformation;
      }
      write_lock.unlock();
      return true;
    }
  } // ScanMatching
} // L3
