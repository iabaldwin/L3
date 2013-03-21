#include "Controller.h"

namespace L3
{
namespace Visualisers
{


std::ostream& operator<<( std::ostream& o, const control_t& t )
{
    //o << t.x << ":" << t.y << ":" << t.z << ":" << t.r << ":" << t.p << ":" << t.q;
    return o;
}

control_t operator+( const control_t& a, const control_t& b )
{
    control_t retval;

    retval.x = a.x + b.x;
    retval.y = a.y + b.y;
    retval.z = a.z + b.z;
    retval.r = a.r + b.r;
    retval.p = a.p + b.p;
    retval.q = a.q + b.q;

    return retval;
}


}
}
