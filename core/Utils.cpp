#include "Utils.h"

namespace L3
{

namespace Utils
{

void localisePoseChain( POSE_SEQUENCE& poses, const Locale& l )
{
    for( POSE_SEQUENCE_ITERATOR it=poses.begin(); it!= poses.end(); it++ )
    {
        (*it).second->x -= l.x;
        (*it).second->y -= l.y;
    }
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

struct accumulator
{
    accumulator() : counter(0), x(0.0), y(0.0), z(0.0)
    {
    }

    int counter;
    double x, y, z;
    void operator()( std::pair< double, L3::Pose*>  p )
    {
        x += p.second->x;
        y += p.second->y;
   
        counter++;
    }

    std::vector<double> centroid()
    {
        std::vector<double> res(2);
        res[0] = x/counter;
        res[1] = y/counter;

        return res;
    }
};

void localisePoseChainToMean( POSE_SEQUENCE& poses )
{
    // Average 
    accumulator a;
    a = std::for_each( poses.begin(), poses.end(), a );
    std::vector<double> centroid = a.centroid();

    POSE_SEQUENCE_ITERATOR it = poses.begin();

    while( it != poses.end() )
    {
        (*it).second->x -= centroid[0];
        (*it).second->y -= centroid[1];
       
        // Regenerate homogeneous - this is poor
        (*it).second->_update();
   
        it++;
    }
}

std::vector< L3::Pose* > posesFromSwathe( SWATHE* s )
{
    std::vector< L3::Pose* > poses;

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
