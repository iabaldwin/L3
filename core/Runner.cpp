#include "Runner.h"

namespace L3
{
        
    void EstimatorRunner::run()
    {
        std::cout.precision( 16 );
   
        L3::Timing::ChronoTimer t;

        t.begin();

        double start_time = current_time;

        double current;
        
        while( running )
        {
            current = start_time + speedup*t.elapsed();

            /*
             *  Update all watchers
             */
            std::for_each( observers.begin(), observers.end(), std::bind2nd( std::mem_fun( &TemporalObserver::update ), current ) );
            
            /*
             *  Do estimation
             */
            this->update( current );
        }

    }

    L3::Predictor predictor;
        
    bool EstimatorRunner::update( double time )
    {
        /*
         *Get the pose from the pose provider
         */
        L3::SE3 predicted = provider->operator()();
        
        *current = predicted;

        /*
         *Update the experience
         */
        experience->update( predicted.X(), predicted.Y() );
        
        swathe_builder->update( time );

        /*
         *  Point cloud generation, projection
         */
        L3::WriteLock point_cloud_lock( projector->cloud->mutex );
        
        projector->project( swathe_builder->swathe );
        L3::transform( projector->cloud, &predicted );  
        
        point_cloud_lock.unlock();
        /*
         *  Estimate
         */
        (*estimator)( projector->cloud, predicted );
    }
}

