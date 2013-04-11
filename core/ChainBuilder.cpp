#include "ChainBuilder.h"

namespace L3
{
    
    LengthEstimator::LengthEstimator() : initialised(false), previous( L3::SE3::ZERO() )
    {
    }

    double LengthEstimator::operator()( L3::SE3 current )
    {
        if ( !initialised )  
        {
            previous = current;
            initialised = true;
            return 0.0;
        }
        else
        {
            double n = L3::Math::norm( previous, current );
            previous = current;
            return n;
        }
    
    }

    std::ostream& operator<<( std::ostream& o, const RECORD& r ) 
    {
        o<< r.first << " ";

        std::cout << r.second << std::endl;

        return o;
    }



}
