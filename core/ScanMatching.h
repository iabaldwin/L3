#ifndef L3_SCAN_MATCHING_H
#define L3_SCAN_MATCHING_H

#include "Core.h"
#include "Datatypes.h"
#include "Datatypes.h"
#include "Iterator.h"

#include "Poco/Thread.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/shared_array.hpp>

namespace L3
{
namespace ScanMatching
{
    class ScanMatcher  
    {
        public:
          
            ScanMatcher() : initialised(false), scan_points(0), putative_points(0)
            {
                cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>);
                cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>);
                final.reset(new pcl::PointCloud<pcl::PointXYZ>);
            }

            virtual bool match(  const std::pair< double,  boost::shared_ptr< L3::LMS151 > > current_scan, Eigen::Matrix4f& transformation ) =0 ;

            boost::shared_array< double > scan;
            int scan_points;
            
            boost::shared_array< double > putative;
            int putative_points;
            
            boost::shared_array< double > registered;

        protected:

            pcl::PointCloud<pcl::PointXYZ>::Ptr final;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
           
            bool initialised;

            double previous_time;
                
    };

    class ICP : public ScanMatcher
    {
        public:
            bool match(  const std::pair< double,  boost::shared_ptr< L3::LMS151 > > current_scan, Eigen::Matrix4f& transformation ) ;
    };

    struct Engine : L3::TemporalObserver, Lockable
    { 
        Engine( L3::ConstantTimeIterator<L3::LMS151>* windower ) : 
            windower(windower),
            previous_update(0.0),
            matcher( new ICP() ),
            current_transformation( Eigen::Matrix4f::Identity() )
        {
        }
    
        virtual ~Engine()
        {
        }

        Eigen::Matrix4f current_transformation;

        std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;

        std::deque< std::pair< double, Eigen::Matrix4f > > trajectory;

        L3::ConstantTimeIterator<L3::LMS151>* windower;
        
        boost::shared_ptr< ScanMatcher > matcher;

        double previous_update;
        
        bool update( double time );
    };

}
}





#endif
