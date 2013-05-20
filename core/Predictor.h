#ifndef L3_PREDICTOR_H
#define L3_PREDICTOR_H

#include "Iterator.h"
#include "Integrator.h"
#include "Datatypes.h"

namespace L3
{

    class Predictor : public L3::TemporalObserver
    {

        public:

            Predictor( L3::Iterator<L3::LHLV>* iterator ) 
                : LHLV_iterator(iterator),
                    previous_update(0.0)
            {
            }

            bool update( double t );

            bool predict( L3::SE3& predicted, L3::SE3& current );

        private:

            L3::Iterator<L3::LHLV>*      LHLV_iterator;

            double previous_update;

    };

}

#endif
