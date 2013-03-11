#include "Datatypes.h"

namespace L3
{
namespace Math
{
    double norm( const L3::SE3& a, const L3::SE3& b )
    {
        return sqrt( pow(a.x - b.x, 2.0 ) + pow(a.y -b.y, 2.0 ) + pow(a.z -b.z, 2.0 ) );
    }
}
}

