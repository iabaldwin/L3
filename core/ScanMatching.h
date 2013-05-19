#ifndef L3_SCAN_MATCHING_H
#define L3_SCAN_MATCHING_H

#include "Core.h"
#include "Datatypes.h"
#include "Datatypes.h"
#include "Iterator.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <boost/shared_array.hpp>

namespace L3
{
namespace ScanMatching
{
    class ScanMatcher  
    {
        public:
          
            ScanMatcher() : initialised(false)
            {
                cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>);
                cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>);
            }

            //virtual bool match(  const L3::LMS151& current_scan ) = 0;
            virtual bool match(  const std::pair< double,  boost::shared_ptr< L3::LMS151 > > current_scan ) = 0;

            boost::shared_array< double > scan;
            int scan_points;
            
            boost::shared_array< double > putative;
            int putative_points;

            std::pair< double, double > instantaneous_velocity;

        protected:

              
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
        
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
           
            bool initialised;

            double previous_time;
                
    };

    class ICP : public ScanMatcher
    {
        public:

            bool match(  const std::pair< double, boost::shared_ptr< L3::LMS151 > >current_scan );

    };

    struct Engine : L3::TemporalObserver, Lockable
    { 
        Engine( L3::ConstantTimeIterator<L3::LMS151>* windower ) : 
            windower(windower)
        {
            matcher.reset( new ICP() );
        }
            
        std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;

        L3::ConstantTimeIterator<L3::LMS151>* windower;
        
        boost::shared_ptr< ScanMatcher > matcher;

        bool update( double t );

    };

}
}





#endif
