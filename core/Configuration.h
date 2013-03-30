#ifndef L3_CONFIGURATION_H
#define L3_CONFIGURATION_H

#include <vector>
#include <map>
#include <sstream>
#include <iterator>
#include <iostream>

#include <boost/filesystem.hpp>
#include <libconfig.h++>

#include "Datatypes.h"
#include "Utils.h"

namespace L3
{
namespace Configuration
{

boost::filesystem::path configurationDirectory( void );

/*
 *Configuration structures
 */
struct LIDARParameters
{
    LIDARParameters()
    {
        transform.resize( 6 );
    }

    std::string             name;
    std::vector<double>     transform;
    double                  frequency;
    bool                    inverted;
};

struct INSParameters
{
};

struct Configuration
{

    Configuration()
    {
    }

    bool load( const std::string& target )
    {
        FILE* f = fopen( target.c_str(), "r"  );
        if ( !f )
            return false;
        
        config.read( f );
   
        return true;
    }

    boost::filesystem::path p;

    libconfig::Config config;
};

struct Mission : Configuration
{
    Mission( const std::string& DATASET )  : dataset(DATASET)
    {
        // Load configuration directory 
        p = L3::Configuration::configurationDirectory();
   
        // Add target
        p /= "missions";
        p /= dataset + ".config";

        if ( !load( p.string() ) )
            throw std::exception();
  
        loadLIDARS();
    }

    std::string dataset;
    std::string description;
    std::string locale;
    std::map< std::string, LIDARParameters > lidars;

    // Estimation
    std::string horizontal;
    std::string declined;

    bool loadLIDARS()
    {
        std::string current_name;

        std::vector< std::string > params;
        params.push_back( ".x" );
        params.push_back( ".y" );
        params.push_back( ".z" );
        params.push_back( ".r" );
        params.push_back( ".p" );
        params.push_back( ".q" );

        for( int i=0;;i++ )
        {
            std::stringstream ss;
            ss <<  "mission.lasers.[" << i << "]";

            std::string lookup = ss.str();

            if (!config.lookupValue( lookup+".name", current_name))
                break;
            else
            {
                LIDARParameters parameters;


                double result;
                for( int j=0; j<6; j++ )
                {
                    if (!config.lookupValue( lookup+".transform"+params[j], result))
                        return false;
                    parameters.transform[j] = result;
                }

                lidars.insert( std::make_pair( current_name, parameters ) );
            }
        }
    }
};

struct Locale : Configuration
{

    Locale( const std::string& locale ) 
    {

        // Load configuration directory 
        p = L3::Configuration::configurationDirectory();
   
        // Add target
        p /= "datums";
        p /= locale + ".config";


        if ( !load( p.string() ) && loadDatum() )
            throw std::exception();
    }

    bool loadDatum()
    {

        std::stringstream ss;
        ss <<  "datum.X.lower";

        if (!config.lookupValue( ss.str(), x))
            return false; 

        ss.clear();
        ss.str(std::string());

        ss <<  "datum.Y.lower";
        if (!config.lookupValue( ss.str(), y))
            return false; 

    }

    double x, y, z;

};

struct Begbroke : Locale
{

    Begbroke() : Locale( "begbroke_datum" )
    {};

};

struct Woodstock : Locale
{
    Woodstock() : Locale( "begbroke_datum" )
    {};
};


/*
 *Conversion
 */
bool convert( const LIDARParameters& params, L3::SE3& calibration );

/*
 *  I/O
 */
std::ostream& operator<<( std::ostream& o, const Mission& mission );
std::ostream& operator<<( std::ostream& o, const std::pair< std::string, LIDARParameters> & parameters );



} // Configuration
} // L3

#endif

