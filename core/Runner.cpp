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

            /*
             *  Update all watchers
             */
            TemporalRunner::update( current_time );

            /*
             *  Recompute swathe
             */
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
       
            *current = oracle->operator()();
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
        *estimated = algorithm->operator()( projector->cloud, predicted );

        return true;
    }
}

