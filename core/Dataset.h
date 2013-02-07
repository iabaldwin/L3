#ifndef L3_DATATYPES_H
#define L3_DATATYPES_H

#include <iostream>
#include <vector>
#include <iterator>
#include <map>
#include <list>
#include <assert.h>
#include <boost/filesystem.hpp>


namespace L3
{

enum  extensionType { INS, LIDAR };

    class Dataset
    {

        public:
            Dataset();
            Dataset( const std::string& target );

            Dataset& list();

            bool validate();

        protected:

            boost::filesystem::directory_entry OxTS;
            std::list<boost::filesystem::directory_entry> LIDARs;
            
            boost::filesystem::path root_path;
            std::map< std::string, extensionType > lookup;
    };

};

#endif
