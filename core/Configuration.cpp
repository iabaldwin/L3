#include "Configuration.h"

#include <iterator>

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
    o << mission.dataset << ":" << std::endl; 
    std::copy( mission.lidars.begin(), mission.lidars.end(), std::ostream_iterator< std::pair< std::string, LIDARParameters > >( o, "\n" ) );

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
