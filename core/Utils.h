#ifndef L3_UTILS_H
#define L3_UTILS_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include "Datatypes.h"
#include <libconfig.h++>

namespace L3
{

namespace UTILS
{



struct Locale 
{
    std::string name;

    float x,y,z;
};

void localisePoseChain( std::vector<L3::Pose*>& poses, const Locale& l );

boost::filesystem::path configurationDirectory( void );


struct BEGBROKE : Locale
{
    FILE* f;

    BEGBROKE()  
    {
        libconfig::Config config;

        boost::filesystem::path p = L3::UTILS::configurationDirectory();

        p /= "begbroke_datum.config";

        f = fopen( p.string().c_str(), "r"  );

        try
        {

            config.read( f );

            double result;
            if (config.lookupValue( "datum.X.lower", result) )
            {
                x = result;
            }

            if (config.lookupValue( "datum.Y.lower", result) )
            {
                y = result;
            }
        }  
        catch( ... )
        {

        }

    }

};


}


}

#endif
