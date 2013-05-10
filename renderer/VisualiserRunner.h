#ifndef L3_VISUALISERS_RUNNER_H
#define L3_VISUALISERS_RUNNER_H

#include "Components.h"
#include "L3.h"

namespace L3
{
namespace Visualisers
{
    struct VisualiserRunner : L3::Visualisers::Leaf, L3::TemporalRunner
    {
        VisualiserRunner( double start_time=0.0 )
            : time(start_time)
        {
        }

        double time;

        void onDraw3D( glv::GLV& g )
        {
            // Update 
            this->update( time += .5 );
        }

    };

}
}

#endif

