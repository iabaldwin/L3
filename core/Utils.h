#ifndef L3_UTILS_H
#define L3_UTILS_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include "Datatypes.h"
#include "Definitions.h"
#include <libconfig.h++>

namespace L3
{

namespace Utils
{

boost::filesystem::path configurationDirectory( void );

struct Locale 
{
    std::string name;

    Locale( const std::string& target )
    {

        boost::filesystem::path p = L3::Utils::configurationDirectory();

        p /= target;

        try
        {
            libconfig::Config config;
        
            f = fopen( p.string().c_str(), "r"  );

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

    FILE* f;
    float x,y,z;

};

struct BEGBROKE : Locale
{
    BEGBROKE() : Locale("begbroke_datum.config") 
    {
    }

};

struct WOODSTOCK : Locale
{
    WOODSTOCK() : Locale("woodstock_datum.config") 
    {
    }

};

void localisePoseChain( std::vector<L3::Pose*>& poses, const Locale& l );
void localisePoseChainToOrigin( std::vector<L3::Pose*>& poses );
void localisePoseChainToMean( std::vector<L3::Pose*>& poses );
std::vector< L3::Pose* > posesFromSwathe( SWATHE* s );

}
}

#endif
