#include "Runner.h"

namespace L3
{

    bool EstimatorRunner::update( double time )
    {

#ifndef NDEBUG
        boost::timer t;
#endif
        swathe_builder->update( time );

        // Get the pose from the pose provider
        L3::SE3 pose = (*provider)();

        // Update the experience
        experience->update( pose.x, pose.y );
        
#ifndef NDEBUG
        std::cout << "Experience\t" << t.elapsed() << std::endl;
#endif
        projector->project( swathe_builder->swathe );

#ifndef NDEBUG
        std::cout << "Projection\t" << t.elapsed() << std::endl;
#endif
        (*estimator)( projector->cloud, L3::SE3::ZERO() );

#ifndef NDEBUG
        std::cout << "Estimation\t" << t.elapsed() << std::endl;
#endif

    }
}

