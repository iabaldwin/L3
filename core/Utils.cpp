#include "Utils.h"

namespace L3
{

namespace Utils
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
    double origin_x = poses.front()->x;
    double origin_y = poses.front()->y;

    std::vector<L3::Pose*>::iterator it=poses.begin();

    while( it != poses.end() )
    {
        (*it)->x -= origin_x;
        (*it)->y -= origin_y;

        (*it)->_update();

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
    void operator()( L3::Pose* p )
    {
        x += p->x;
        y += p->y;
   
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

void localisePoseChainToMean( std::vector<L3::Pose*>& poses )
{
    // Average 
    accumulator a;
    a = std::for_each( poses.begin(), poses.end(), a );
    std::vector<double> centroid = a.centroid();

    std::vector<L3::Pose*>::iterator it = poses.begin();

    while( it != poses.end() )
    {
        (*it)->x -= centroid[0];
        (*it)->y -= centroid[1];
       
        // Regenerate homogeneous - this is poor
        (*it)->_update();
   
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
