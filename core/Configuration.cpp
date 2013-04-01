#include "Configuration.h"

#include <iterator>
#include <sstream>

namespace L3
{
namespace Configuration
{

/*
 *Utility functions
 */
boost::filesystem::path configurationDirectory( void )
{
    char * pPath;
    pPath = getenv ("HOME");

    if (pPath==NULL)
        return boost::filesystem::path();

    boost::filesystem::path p;
    p /= std::string( pPath );

    p /= "code";
    p /= "datasets";
    p /= "configuration";

    if ( !boost::filesystem::exists(p) || !boost::filesystem::is_directory( p ) )
        return boost::filesystem::path();
    else
        return p;
}


bool Mission::loadLIDARS()
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

            for( int j=0; j<6; j++ )
            {
                double dresult;
                if (!config.lookupValue( lookup+".transform"+params[j], dresult))
                {
                    int iresult;
                    if( !config.lookupValue( lookup+".transform"+params[j], iresult ) )
                        return false;
                    else
                    parameters.transform[j] = (double)iresult;

                }
                else
                    parameters.transform[j] = dresult;

            }

            lidars.insert( std::make_pair( current_name, parameters ) );
        }
    }

    return lidars.size() > 0;
}

bool Mission::loadDescription()
{
    return config.lookupValue( "mission.description", description);
}

bool Mission::loadLocale()
{
    std::string locale;
    return config.lookupValue( "mission.locale", locale);
}

bool Mission::loadEstimation()
{
    return ( config.lookupValue( "estimation.laser.declined", declined )  &&
            config.lookupValue( "estimation.laser.horizontal", horizontal ) );
}



/*
 *  I/O
 */
std::ostream& operator<<( std::ostream& o, const LIDARParameters& parameters )
{
    o << parameters.name << std::endl;
    o << "\t"; std::copy( parameters.transform.begin(), parameters.transform.end(), std::ostream_iterator<double>( o, " " ) );

    return o;
}

std::ostream& operator<<( std::ostream& o, const std::pair< std::string, LIDARParameters> & parameters )
{
    o << parameters.first << ":";
    o << parameters.second;

    return o;
}

std::ostream& operator<<( std::ostream& o, const Mission& mission )
{
    std::stringstream ss;
    for( std::string::const_iterator it = mission.target.begin(); it != mission.target.end(); it++ )
        ss << "-";

    o << ss.str() << std::endl << mission.target << ":" << std::endl; 
    std::copy( mission.lidars.begin(), mission.lidars.end(), std::ostream_iterator< std::pair< std::string, LIDARParameters > >( o, "\n" ) );

    o << "Declined :" <<  mission.declined << std::endl;
    o << "Inclinded :" <<  mission.declined << std::endl;

    return o;
}

/*
 *Conversion
 */
bool convert( const LIDARParameters& params, L3::SE3& calibration )
{
    std::vector<double> transform = params.transform;
 
    for( int i=3; i<transform.size(); i++ )
        transform[i] = L3::Utils::Math::degreesToRadians( transform[i] );
    
    calibration = L3::SE3( transform );
}

}
}
