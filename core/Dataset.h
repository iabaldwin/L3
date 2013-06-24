#ifndef L3_DATASET_H
#define L3_DATASET_H

#include <iostream>
#include <iterator>
#include <vector>
#include <map>
#include <list>
#include <assert.h>

#include <boost/filesystem.hpp>
#include <boost/noncopyable.hpp>

#include "Poco/Runnable.h"
#include "Poco/Thread.h"

#include "Timing.h"
#include "WindowerFactory.h"

namespace L3
{

enum extensionType { INS_file=1, LIDAR_file, LHLV_file, SM_file };

class Dataset : private boost::noncopyable
{

    public:
        Dataset( const std::string& target );
        ~Dataset();

        bool            validate();
        bool            load();
        std::string     path() const { return root_path.string(); };
        std::string     name() const { return dataset_name;};

        double          start_time;

        // Lookups
        std::vector<std::string>    LIDAR_names;

        // Threaded windowers
        boost::shared_ptr<SlidingWindow<L3::SE3> >                  pose_reader;
        boost::shared_ptr<SlidingWindow<L3::LHLV> >                 LHLV_reader;
        boost::shared_ptr<SlidingWindow<L3::SMVelocity> >           velocity_reader;
        std::map< std::string, boost::shared_ptr< SlidingWindow<L3::LMS151> > > LIDAR_readers;
       
        
    protected:
        
        std::list< boost::shared_ptr< Poco::Runnable> >  runnables;
        std::list< boost::shared_ptr< Poco::Thread > >   threads;
        
        friend std::ostream& operator<<( std::ostream& o, const Dataset& dataset );

        boost::filesystem::directory_entry              OxTS_ins;
        boost::filesystem::directory_entry              OxTS_lhlv;
        boost::filesystem::directory_entry              SM_vel;
        std::list<boost::filesystem::directory_entry>   LIDARs;
        
        boost::filesystem::path root_path;
        std::map< std::string, extensionType > lookup;

        std::string dataset_name;
};

} // L3

#endif
