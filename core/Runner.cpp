#include "Runner.h"

namespace L3
{

    void EstimatorRunner::run()
    {
        L3::Tools::Timer t;

        std::cout.precision( 16 );

        double start_time = current_time;

        while( running )
        {
            double current_time = start_time + t.elapsed();

            this->update( current_time );
        }

    }

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
        L3::WriteLock point_cloud_lock( projector->cloud->mutex );
        projector->project( swathe_builder->swathe );
        point_cloud_lock.unlock();

#ifndef NDEBUG
        std::cout << "Projection\t" << t.elapsed() << std::endl;
#endif
      
        (*estimator)( projector->cloud, (*provider)() );


#ifndef NDEBUG
        std::cout << "Estimation\t" << t.elapsed() << std::endl;
#endif

    }
}

