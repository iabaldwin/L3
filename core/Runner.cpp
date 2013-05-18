#include "Runner.h"

namespace L3
{

    DatasetRunner::DatasetRunner( L3::Dataset* dataset, L3::Configuration::Mission* mission, float speedup ) 
        : dataset(dataset), 
            speedup(speedup),
            current_time(0.0),
            start_time(dataset->start_time)
    {
        // Constant time iterator over poses
        pose_iterator.reset( new L3::ConstantTimeIterator<L3::SE3>( dataset->pose_reader ) );
        LHLV_iterator.reset( new L3::ConstantTimeIterator<L3::LHLV> ( dataset->LHLV_reader ) );  
       
        // LIDAR iterators
        horizontal_LIDAR.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers[ mission->horizontal] ));
        vertical_LIDAR.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers[ mission->declined] ));

        // Point-clouds
        point_cloud.reset( new L3::PointCloud<double>() );
        projection.reset( new L3::SE3( L3::SE3::ZERO() ) );
        
        L3::Configuration::convert( mission->lidars[mission->declined], *projection );

        projector.reset( new L3::Projector<double>( projection.get(), point_cloud.get() ) );
       
        // Pose Provider
        pose_windower.reset( new L3::ConstantTimeWindower< L3::LHLV>( LHLV_iterator.get() ) );
       
        // Swathe generator
        swathe_builder.reset( new L3::SwatheBuilder( pose_windower.get(), vertical_LIDAR.get() ) );
   
        // INS pose
        oracle.reset( new L3::ConstantTimeWindower< L3::SE3 >( pose_iterator.get() ) );

        // Scan matching engine
        engine.reset( new L3::ScanMatching::Engine( horizontal_LIDAR.get() ) );
        
        (*this)<< pose_iterator.get() << LHLV_iterator.get() << vertical_LIDAR.get() << horizontal_LIDAR.get() << engine.get() << pose_windower.get() << swathe_builder.get();

        current.reset( new L3::SE3( L3::SE3::ZERO() ) );
    
        //this->provider = oracle.get();
        this->provider = oracle;
    }
    
    /*
     *  Dataset runner
     */

    void DatasetRunner::run()
    {
        L3::Timing::ChronoTimer t;

        t.begin();

        double real_time_elapsed = t.elapsed();

        current_time = start_time;

        while( running )
        {
            current_time += ( t.elapsed() - real_time_elapsed )*speedup;
                
            real_time_elapsed = t.elapsed(); 

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
         *  Update the experience
         */
        experience->update( predicted.X(), predicted.Y() );

        /*
         *  Estimate
         */
       
        *estimated = algorithm->operator()( projector->cloud, predicted );

        return true;
    }
}

