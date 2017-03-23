#ifndef L3_CHAIN_BUILDER_H
#define L3_CHAIN_BUILDER_H

#include <cmath>
#include <numeric>
#include <iterator>

#include "Core.h"
#include "Iterator.h"
#include "Datatypes.h"
#include "Integrator.h"
#include "VelocityProvider.h"

namespace L3
{

class ChainBuilder 
{
    public:

        ChainBuilder(L3::VelocityProvider* provider) : velocity_provider(provider) {
        }

        bool update() {
            // Reset
            window.clear();

            // Allocate
            window.resize(velocity_provider->filtered_velocities.size());

            _incremental_distances.resize(velocity_provider->filtered_velocities.size());

            double distance;

            // Accumulate 
            L3::trajectoryAccumulate(velocity_provider->filtered_velocities.begin(), 
                                        velocity_provider->filtered_velocities.end(), 
                                        window.begin(), 
                                        distance, 
                                        std::numeric_limits<double>::infinity());

            return true;
        }

        std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > window;

    private:

        std::deque < double > _incremental_distances;
        L3::VelocityProvider* velocity_provider;

};

} // namespace L3

#endif
