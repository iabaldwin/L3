#ifndef L3_DATASET_H
#define L3_DATASET_H

#include <iostream>
#include <iterator>
#include <vector>
#include <map>
#include <list>
#include <assert.h>
#include <boost/filesystem.hpp>

#include "Poco/Runnable.h"
#include "Poco/Thread.h"

#include "Datatypes.h"
#include "Tools.h"
#include "Windower.h"

namespace L3
{

enum extensionType { INS_file, LIDAR_file, LHLV_file };

class Dataset
{

    public:
        Dataset( const std::string& target );
        ~Dataset();

        bool            validate();
        bool            load();
        std::string     path(){ return root_path.string(); };

        // Windowers
        SlidingWindow<L3::Pose>*                pose_reader;
        SlidingWindow<L3::LHLV>*                LHLV_reader;
        std::list< SlidingWindow<L3::LIDAR>*>   LIDAR_readers;
       
        // Lookups
        std::vector<std::string>                LIDAR_names;

    protected:
        
        std::list< Poco::Runnable* >            runnables;
        std::list< Poco::Thread* >              threads;
        
        friend std::ostream& operator<<( std::ostream& o, const Dataset& dataset );

        boost::filesystem::directory_entry              OxTS_ins;
        boost::filesystem::directory_entry              OxTS_lhlv;
        std::list<boost::filesystem::directory_entry>   LIDARs;
        
        boost::filesystem::path root_path;
        std::map< std::string, extensionType > lookup;
};

} // L3

#endif
