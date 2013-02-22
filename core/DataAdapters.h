#ifndef L3_ADAPTERS_H
#define L3_ADAPTERS_H

#include <iostream>
#include <fstream>

#include "PointCloud.h"

namespace L3
{
    template <typename T>
    bool writePCLASCII( const std::string& output, PointCloud<T>& cloud)
    {
        std::ofstream writer( output.c_str() );

        if ( !writer.good() )
            return false;
        
        writer << "VERSION .5" << std::endl;
        writer << "FIELDS x y z" << std::endl;
        writer << "SIZE 4 4 4 4" << std::endl;
        writer << "TYPE F F F F" << std::endl;
        writer << "COUNT 1 1 1 1" << std::endl;
        writer << "WIDTH " << cloud.size() << std::endl;
        writer << "HEIGHT 1" << std::endl;
        writer << "POINTS " << cloud.size() << std::endl;
        writer << "DATA ascii" << std::endl;

        // Write cloud
        writer << cloud;

        // Done
        writer.close();

        return true;
    }

}

#endif
