#include "ScanMatching.h"

int do_projection( const L3::LMS151& current_scan, double* matrix, double threshold = 5.0 )
{
    double* matrix_ptr = matrix;

    double range,angle,x,y,z=0;
  
    int counter = 0;

    for (int scan_counter=0; scan_counter<541; scan_counter++) 
    {
        // Compute angle 
        angle = scan_counter*current_scan.angle_spacing +  current_scan.angle_start; 
        range = current_scan.ranges[scan_counter];  

        if ( range < threshold )
            continue;

        x = range*cos( angle );
        y = range*sin( angle );
        
        *matrix_ptr++ = x; 
        *matrix_ptr++ = y; 
        *matrix_ptr++ = z; 
 
        counter++;
    }

    return counter;
}

namespace L3
{
namespace ScanMatching
{
    bool ICP::match(  const L3::LMS151& current_scan )
    {
        if ( !initialised )
        {
            scan.reset( new double[541*3] );
            scan_points = do_projection( current_scan, scan.get() );

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

        for( int i=0; i< scan_points; i++ )
        {
            cloud_in->points[i].x = *cloud_ptr++;
            cloud_in->points[i].y = *cloud_ptr++;
            cloud_in->points[i].z = *cloud_ptr++;
        }


        /*
         *  Putative
         */
        putative.reset( new double[541*3] );
        putative_points  = do_projection( current_scan, putative.get() );

        cloud_out->width    = putative_points;
        cloud_out->height   = 1;
        cloud_out->is_dense = false;

        cloud_out->points.resize (cloud_out->width * cloud_out->height);

        cloud_ptr = putative.get();

        for( int i=0; i< putative_points; i++ )
        {
            cloud_out->points[i].x = *cloud_ptr++;
            cloud_out->points[i].y = *cloud_ptr++;
            cloud_out->points[i].z = *cloud_ptr++;
        }

        icp.setInputCloud(cloud_in);
        icp.setInputTarget(cloud_out);
        pcl::PointCloud<pcl::PointXYZ> Final;

        icp.align(Final);

        Eigen::Matrix4f transformation * icp.getFinalTransformation();

        // TAke in a pair, instead of just a scan - need the time
        //double vel = sqrt( pow( transformation( 0,3 ), 2 ) + pow( transformation( 1,3 ), 2 ) )/

        // Do swap
        scan.swap( putative );
        scan_points = putative_points;
   
        return true;
    }
    
    bool Engine::update( double time )
    {
        this->windower->getWindow( window );

        bool retval = false;

        if ( window.size() > 0 )
        {
            L3::WriteLock( this->mutex );
            retval = matcher->match( *(window.back().second ) ); 
        }
  
        return retval;
    }


}
}
