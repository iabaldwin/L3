#ifndef L3_SCAN_MATCHING_H
#define L3_SCAN_MATCHING_H

#include "Core.h"
#include "Datatypes.h"
#include "Datatypes.h"
#include "Iterator.h"
#include "Filter.h"

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
          
            ScanMatcher() : scan_points(0), putative_points(0), initialised(false)
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
            current_update(0.0),
            matcher( new ICP() ),
            current_transformation( Eigen::Matrix4f::Identity() )
        {
            _linear_velocity_filter = boost::make_shared< L3::Estimator::AlphaBetaFilter >(.05,0.0001);
            _rotational_velocity_filter = boost::make_shared< L3::Estimator::AlphaBetaFilter >(.05,0.0001);
       
            raw_velocity_data.second.resize( 4);
            filtered_velocity_data.second.resize( 4);
        }
    
        boost::shared_ptr< L3::Estimator::AlphaBetaFilter > _linear_velocity_filter;
        boost::shared_ptr< L3::Estimator::AlphaBetaFilter > _rotational_velocity_filter;

        std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;

        std::deque< std::pair< double, Eigen::Matrix4f > > trajectory;

        L3::ConstantTimeIterator<L3::LMS151>* windower;
        
        double previous_update, current_update;
        
        boost::shared_ptr< ScanMatcher > matcher;
        
        Eigen::Matrix4f current_transformation, previous_transformation;
            
        std::pair< double, std::vector<double> > raw_velocity_data, filtered_velocity_data;
        
        bool update( double time );
    };

}
}





#endif
