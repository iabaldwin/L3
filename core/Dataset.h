#ifndef L3_DATASET_H
#define L3_DATASET_H

#include <iostream>
#include <iterator>
#include <vector>
#include <map>
#include <list>
#include <assert.h>
#include <boost/filesystem.hpp>
#include "Reader.h"

namespace L3
{

enum  extensionType { INS, LIDAR };

    class Dataset
    {

        public:
            Dataset();
            Dataset( const std::string& target );

            bool validate();
            bool load();

            std::vector<L3::Pose*> poses;

            std::vector<std::string> LIDAR_names;
            std::map< std::string, std::vector<L3::LMS151*> > LIDAR_data;

            Pose*   getPoseAtTime( double time );
            LMS151* getScanAtTime( double time, const std::string& name );

            //DBG
            bool    isSorted()
            {
           
                std::vector<L3::Pose*>::iterator it = poses.begin();

                it++;
                while( it != poses.end() )
                {
                if ( (*(it-1))->time > (*it)->time )
                        return false;
                    it++;
                }
                return true;

            };

        protected:
            
            friend std::ostream& operator<<( std::ostream& o, const Dataset& dataset );

            boost::filesystem::directory_entry OxTS;
            std::list<boost::filesystem::directory_entry> LIDARs;
            
            boost::filesystem::path root_path;
            std::map< std::string, extensionType > lookup;
    };

};

#endif
