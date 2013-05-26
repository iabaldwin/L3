#ifndef L3_INTEGRATOR_H
#define L3_INTEGRATOR_H

#include "Core.h"
#include "Datatypes.h"

namespace L3
{

    struct LengthEstimator 
    {

        LengthEstimator() : initialised(false), previous( L3::SE3::ZERO() )
        {
        }

        bool initialised;
        L3::SE3 previous;

        double operator()( L3::SE3 current )
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
    };

    template <typename InputIterator, typename OutputIterator1, typename OutputIterator2 >
        double trajectoryAccumulate( InputIterator begin, InputIterator end, OutputIterator1 output, OutputIterator2 distances );

    template <typename InputIterator, typename OutputIterator>
        double swatheLength( InputIterator begin, InputIterator end, OutputIterator output );



}

#endif

