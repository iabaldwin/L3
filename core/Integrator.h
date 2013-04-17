#ifndef L3_INTEGRATOR_H
#define L3_INTEGRATOR_H

#include "Core.h"
#include "Datatypes.h"

namespace L3
{

    struct LengthEstimator 
    {

        LengthEstimator();

        bool initialised;
        L3::SE3 previous;

        double operator()( L3::SE3 current );

    };

    template <typename InputIterator, typename OutputIterator >
        void trajectoryAccumulate( InputIterator begin, InputIterator end, OutputIterator output );
        
}

#endif

