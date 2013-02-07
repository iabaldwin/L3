#include "Utils.h"

namespace L3
{

namespace UTILS
{

void localisePoseChain( std::vector<L3::Pose*>& poses, const Locale& l )
{
    for( std::vector<L3::Pose*>::iterator it=poses.begin(); it!= poses.end(); it++ )
    {
        (*it)->x -= l.x;
        (*it)->y -= l.y;
    }

}

void localisePoseChainToOrigin( std::vector<L3::Pose*>& poses )
{
    //L3::Pose* origin = poses.front();
    double origin_x = poses.front()->x;
    double origin_y = poses.front()->y;

    std::vector<L3::Pose*>::iterator it=poses.begin();

    while( it != poses.end() )
    {
        (*it)->x -= origin_x;
        (*it)->y -= origin_y;
   
        it++;
    }

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


}
}
