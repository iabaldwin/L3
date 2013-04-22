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

    L3::Predictor predictor;

    bool EstimatorRunner::update( double time )
    {
        // Get the pose from the pose provider
        L3::SE3 predicted = provider->operator()();
        //*current = L3::SE3::ZERO();
        *current = predicted;

        // Update the experience
        experience->update( predicted.X(), predicted.Y() );
        
        //predictor.predict( predicted, 
                    //current, 
                    //provider->constant_time_iterator->window.begin(), 
                    //provider->constant_time_iterator->window.end() );

        swathe_builder->update( time );

        /*
         *  Point cloud generation, projection
         */
        L3::WriteLock point_cloud_lock( projector->cloud->mutex );
        projector->project( swathe_builder->swathe );
        L3::transform( projector->cloud, &predicted );  
        point_cloud_lock.unlock();

     
        /*
         *Estimation
         */
        (*estimator)( projector->cloud, predicted );

    }
}

