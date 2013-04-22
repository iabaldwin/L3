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

double radiansToDegrees( double radians )
{
        return (180.0/M_PI)*radians;
}


} // Math

void localisePoseChainToOrigin( POSE_SEQUENCE& poses )
{

    //double origin_x = poses.front().second->x;
    //double origin_y = poses.front().second->y;

    //POSE_SEQUENCE_ITERATOR it = poses.begin();

    //while( it != poses.end() )
    //{
        //(*it).second->x -= origin_x;
        //(*it).second->y -= origin_y;

        //(*it).second->_update();

        //it++;
    //}
}



} // Utils
} // L3
