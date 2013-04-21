#include "Runner.h"

namespace L3
{
    void EstimatorRunner::run()
    {
        L3::Tools::Timer t;

        double start_time = current_time;

        std::cout.precision( 16 );

        double current;

        while( running )
        {
            current = start_time + t.elapsed();

            /*
             *Update all watchers
             */
            for( std::list < TemporalObserver* >::iterator it = observers.begin(); 
                    it != observers.end(); 
                    it++ )
                (*it)->update( current );

            /*
             *Do estimation
             */
            this->update( current );
        }

    }

    bool EstimatorRunner::update( double time )
    {

//#ifndef _NDEBUG
        boost::timer t;
//#endif
        swathe_builder->update( time );

        // Get the pose from the pose provider
        L3::SE3 pose = provider->operator()();

        // Update the experience
        experience->update( pose.X(), pose.Y() );

//#ifndef _NDEBUG
        //std::cout << "Experience\t" << t.elapsed() << std::endl;
//#endif
        
        L3::WriteLock point_cloud_lock( projector->cloud->mutex );
        projector->project( swathe_builder->swathe );
        L3::transform( projector->cloud, &pose );
        point_cloud_lock.unlock();



//#ifndef _NDEBUG
        //std::cout << "Projection\t" << t.elapsed() << std::endl;
//#endif
      
        //(*estimator)( projector->cloud, (*provider)() );

#ifndef _NDEBUG
        std::cout << "Estimation\t" << t.elapsed() << std::endl;
#endif

    }
}

