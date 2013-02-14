#ifndef L3_DATASET_H
#define L3_DATASET_H

#include <iostream>
#include <iterator>
#include <vector>
#include <map>
#include <list>
#include <assert.h>
#include <boost/filesystem.hpp>

#include "Datatypes.h"
#include "Tools.h"
#include "Reader.h"

namespace L3
{

enum extensionType { INS_file, LIDAR_file, LHLV_file };

class Dataset
{

    public:
        Dataset();
        Dataset( const std::string& target );

        bool        validate();
        bool        load();
        std::string path(){ return root_path.string(); };

        // Scans & poses
        std::vector<L3::Pose*>                              poses;
        std::vector<L3::LHLV*>                              LHLV_data;
        std::vector<std::string>                            LIDAR_names;
        std::map< std::string, std::vector<L3::LMS151*> >   LIDAR_data;

        // Helper functions
        Pose*   getPoseAtTime( double time );
        LMS151* getScanAtTime( double time, const std::string& name );

    protected:
        
        friend std::ostream& operator<<( std::ostream& o, const Dataset& dataset );

        boost::filesystem::directory_entry OxTS_ins;
        boost::filesystem::directory_entry OxTS_lhlv;
        std::list<boost::filesystem::directory_entry> LIDARs;
        
        boost::filesystem::path root_path;
        std::map< std::string, extensionType > lookup;

};

} // L3

#endif
