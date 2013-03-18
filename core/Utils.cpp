#include "Utils.h"

namespace L3
{
namespace Utils
{
namespace Math
{

double degreesToRadians( double degrees )
{
        return (M_PI/180.0)*degrees;
}

boost::filesystem::path configurationDirectory( void )
{
 char * pPath;
    pPath = getenv ("HOME");
    
    if (pPath==NULL)
        return boost::filesystem::path();

    boost::filesystem::path p;
    p /= std::string( pPath );

    p /= "code";
    p /= "matlab";
    p /= "conf";

    if ( !boost::filesystem::exists(p) || !boost::filesystem::is_directory( p ) )
        return boost::filesystem::path();
    else
        return p;
   
}

L3::SE3 loadCalibration( const std::string& target, const::std::string& LIDAR )
{
    L3::SE3 retval = L3::SE3::ZERO();

    boost::filesystem::path p = L3::Utils::configurationDirectory();

    p /= target;

    libconfig::Config config;

    FILE* f = fopen( p.string().c_str(), "r"  );

    config.read( f );

    for( int i=0;;i++ )
    {
        std::stringstream ss;
        ss <<  "mission.lasers.[" << i << "]";
        std::string lookup = ss.str();

        std::string current_name;
        if (!config.lookupValue( lookup+".name", current_name))
            throw L3::calibration_failure();

        if (strcmp( current_name.c_str(), LIDAR.c_str() ) == 0)
        {
            double result;
            if (!config.lookupValue( lookup+".transform.x", result))
            {
                std::cout << lookup+".transform.x" << std::endl;

                    std::cout << "X" << std::endl;

                throw L3::calibration_failure();
            }
            retval.x = result;

            if( !config.lookupValue( lookup+".transform.y", result))
            {
                std::cout << "Y" << std::endl;
                throw L3::calibration_failure();
            }
            retval.y = result;

            if( !config.lookupValue( lookup+".transform.z", result))
                throw L3::calibration_failure();
            retval.z = result;

            if( !config.lookupValue( lookup+".transform.r", result))
                throw L3::calibration_failure();
            retval.r = L3::Utils::Math::degreesToRadians( result );
    
            if( !config.lookupValue( lookup+".transform.p", result))
                throw L3::calibration_failure();
            retval.p = L3::Utils::Math::degreesToRadians(  result );

            if( !config.lookupValue( lookup+".transform.q", result))
                throw L3::calibration_failure();
            retval.q = L3::Utils::Math::degreesToRadians( result );

            break;
        }
        else
            continue;
    }
   
    retval._update();
    return retval;

}

void localisePoseChainToOrigin( POSE_SEQUENCE& poses )
{

    double origin_x = poses.front().second->x;
    double origin_y = poses.front().second->y;

    POSE_SEQUENCE_ITERATOR it = poses.begin();

    while( it != poses.end() )
    {
        (*it).second->x -= origin_x;
        (*it).second->y -= origin_y;

        (*it).second->_update();

        it++;
    }
}



} // Math
} // Utils
} // L3
