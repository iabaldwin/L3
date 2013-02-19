#include "Utils.h"

namespace L3
{

namespace Utils
{

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

std::vector< boost::shared_ptr<L3::Pose> > posesFromSwathe( SWATHE* s )
{
    std::vector< boost::shared_ptr<L3::Pose> > poses;

    SWATHE::iterator it = s->begin();

    while( it != s->end() )
    {
        poses.push_back( (*it).first );
   
        it++;
    }

    return poses;
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

} // Utils
} // L3
