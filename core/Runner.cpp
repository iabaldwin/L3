#include "Runner.h"

namespace L3
{

    /*
     *  Dataset runner
     */

    void DatasetRunner::run()
    {
        L3::Timing::ChronoTimer t;

        t.begin();

        while( running )
        {
            current_time = start_time + speedup*t.elapsed();

            TemporalRunner::update( current_time );

            /*
             *  Update all watchers
             */
            //std::for_each( observers.begin(), observers.end(), std::bind2nd( std::mem_fun( &TemporalObserver::update ), current_time ) );
    
            swathe_builder->update( current_time );

            /*
             *  Point cloud generation, projection
             */
            L3::WriteLock point_cloud_lock( projector->cloud->mutex );
            projector->project( swathe_builder->swathe );
            point_cloud_lock.unlock();

            /*
             *  Update everything else
             */
            update( current_time );
        
        }

    }
   
    bool EstimatorRunner::update( double time )
    {

        /*
         *  Get the pose from the pose provider
         */
        L3::SE3 predicted = provider->operator()();

        *current = predicted;

        /*
         *Update the experience
         */
        experience->update( predicted.X(), predicted.Y() );


        /*
         *Estimate
         */
        *estimated = estimator->operator()( projector->cloud, predicted );

        return true;
    }
}

