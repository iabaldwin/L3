#ifndef L3_CHAIN_BUILDER_H
#define L3_CHAIN_BUILDER_H

#include <cmath>
#include <numeric>
#include <iterator>

#include "Datatypes.h"
#include "Core.h"
#include "Iterator.h"
#include "Integrator.h"

namespace L3
{
    class ChainBuilder 
    {
        public:

            ChainBuilder( L3::Iterator<L3::LHLV>* iterator ) : LHLV_iterator(iterator)
            {
            }

            bool update()
            {
                // Reset
                window.clear();

                // Allocate
                window.resize( LHLV_iterator->window.size() );

                _incremental_distances.resize( window.size() );

                double distance;

                // Accumulate 
                L3::trajectoryAccumulate( LHLV_iterator->window.begin(), 
                                            LHLV_iterator->window.end(), 
                                            window.begin(), 
                                            distance, 
                                            std::numeric_limits<double>::infinity() );

                return true;
            }

            std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > window;

        private:

            std::deque < double > _incremental_distances;

            L3::Iterator<L3::LHLV>* LHLV_iterator;

    };

}

#endif
