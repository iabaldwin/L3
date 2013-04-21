#ifndef L3_PREDICTOR_H
#define L3_PREDICTOR_H

#include "Integrator.h"
#include "Misc.h"

namespace L3
{

    struct Predictor
    {
        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > chain;
        
        template <typename InputIterator >
            bool predict( L3::SE3& predicted, L3::SE3& current, InputIterator start, InputIterator end );
        
    };

}

#endif
