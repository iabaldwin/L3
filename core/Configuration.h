#ifndef L3_CONFIGURATION_H
#define L3_CONFIGURATION_H

#include <vector>
#include <map>
#include <sstream>
#include <iterator>

#include <boost/filesystem.hpp>
#include <libconfig.h++>

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
    }

    boost::filesystem::path p;

    libconfig::Config config;
};

std::ostream& operator<<( std::ostream& o, const std::pair< std::string, LIDARParameters> & parameters );

struct Mission : Configuration
{
    Mission( const std::string& dataset ) 
    {
        // Load configuration directory 
        p = L3::Configuration::configurationDirectory();
   
        // Add target
        p /= "missions";
        p /= dataset + ".config";

        if ( !load( p.string() ) )
            throw std::exception();
  

        std::cout << p << std::endl;

        loadLIDARS();
   
        //std::copy( lidars.begin(), lidars.end(), std::ostream_iterator<LIDARParameters>( std::cout, " " ) );
        std::copy( lidars.begin(), lidars.end(), std::ostream_iterator< std::pair< std::string, LIDARParameters > >( std::cout, " " ) );
    }

    std::string  description;
    std::string  locale;
    std::map< std::string, LIDARParameters > lidars;

    // Estimation
    std::string horizontal;
    std::string declined;

    bool loadLIDARS()
    {
        for( int i=0;;i++ )
        {
            std::stringstream ss;
            ss <<  "mission.lasers.[" << i << "]";
            
            std::string lookup = ss.str();

            std::string current_name;
   
            std::cout << ss.str() << std::endl;

            if (!config.lookupValue( lookup+".name", current_name))
                break;
            else
            {
                LIDARParameters parameters;

                std::vector< std::string > params;
                params.push_back( ".x" );
                params.push_back( ".y" );
                params.push_back( ".z" );
                params.push_back( ".r" );
                params.push_back( ".p" );
                params.push_back( ".q" );

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
    }

    double x, y, z;

};


} // Configuration
} // L3

#endif

