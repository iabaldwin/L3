#include "Runner.h"

namespace L3
{
        
    void EstimatorRunner::run()
    {
        std::cout.precision( 16 );
   
        L3::Timing::ChronoTimer t;

        t.begin();

        while( running )
        {
            current_time = start_time + speedup*t.elapsed();

            /*
             *  Update all watchers
             */
            std::for_each( observers.begin(), observers.end(), std::bind2nd( std::mem_fun( &TemporalObserver::update ), current_time ) );
            
            /*
             *  Do estimation
             */
            this->update( current_time );
        }

    }

    L3::Predictor predictor;
        
    bool EstimatorRunner::update( double time )
    {
        /*
         *  Get the pose from the pose provider
         */
        L3::SE3 predicted = provider->operator()();
        
        *current = predicted;

        /*
         *  Update the experience
         */
        experience->update( predicted.X(), predicted.Y() );
        
        swathe_builder->update( time );

        /*
         *  Point cloud generation, projection
         */
        L3::WriteLock point_cloud_lock( projector->cloud->mutex );
        projector->project( swathe_builder->swathe );
        point_cloud_lock.unlock();
    
        /*
         *  Estimate
         */
        *estimated = estimator->operator()( projector->cloud, predicted );

    }
}

