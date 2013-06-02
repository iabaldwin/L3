#ifndef L3_SCAN_MATCHING_H
#define L3_SCAN_MATCHING_H

#include "Core.h"
#include "Datatypes.h"
#include "Datatypes.h"
#include "Iterator.h"

#include "Poco/Thread.h"

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

            virtual bool match(  const std::pair< double,  boost::shared_ptr< L3::LMS151 > > current_scan, Eigen::Matrix4f& transformation ) =0 ;

            boost::shared_array< double > scan;
            int scan_points;
            
            boost::shared_array< double > putative;
            int putative_points;

            std::pair< double, double > instantaneous_velocity;

        protected:

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
            pcl::PointCloud<pcl::PointXYZ>::Ptr final;
           
            bool initialised;

            double previous_time;
                
    };

    class ICP : public ScanMatcher
    {
        public:

            bool match(  const std::pair< double,  boost::shared_ptr< L3::LMS151 > > current_scan, Eigen::Matrix4f& transformation ) ;
    };

    struct Engine : Lockable, Poco::Runnable
    { 
        Engine( L3::ConstantTimeIterator<L3::LMS151>* windower ) : 
            windower(windower),
            running(true)
        {
            matcher.reset( new ICP() );
       
            current_transformation = Eigen::Matrix4f::Identity();
       
            thread.start( *this );
        }
    
        virtual ~Engine()
        {
            running = false;;
            if ( thread.isRunning() )
                thread.join();
        }

        Poco::Thread thread;

        bool running;

        Eigen::Matrix4f current_transformation;

        std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;

        L3::ConstantTimeIterator<L3::LMS151>* windower;
        
        boost::shared_ptr< ScanMatcher > matcher;

        std::deque< Eigen::Matrix4f > trajectory;

        bool update( double t );

        void run();
    };

}
}





#endif
