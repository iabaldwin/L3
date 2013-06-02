#include "Runner.h"

namespace L3
{

    DatasetRunner::DatasetRunner( L3::Dataset* dataset, L3::Configuration::Mission* mission, float speedup ) 
        : dataset(dataset), 
            mission(mission),
            speedup(speedup),
            current_time(0.0),
            start_time(dataset->start_time),
            stand_alone(false)
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

        // Get the calibratoin
        L3::Configuration::convert( mission->lidars[mission->declined], *projection );

        projector.reset( new L3::Projector<double>( projection.get(), point_cloud.get() ) );
       
        // Pose Provider
        //pose_windower.reset( new L3::ConstantTimeWindower< L3::LHLV>( LHLV_iterator.get() ) );
        pose_windower.reset( new L3::ConstantDistanceWindower( LHLV_iterator.get(), 40 ) );
       
        // Swathe generator
        //swathe_builder.reset( new L3::RawSwatheBuilder( pose_windower.get(), vertical_LIDAR.get() ) );
        swathe_builder.reset( new L3::BufferedSwatheBuilder( pose_windower.get(), vertical_LIDAR.get() ) );
   
        // INS pose
        oracle.reset( new L3::ConstantTimeWindower< L3::SE3 >( pose_iterator.get() ) );

        // Scan matching engine
        engine.reset( new L3::ScanMatching::Engine( horizontal_LIDAR.get() ) );
       
        // Predictor
        predictor.reset( new L3::Predictor( LHLV_iterator.get() ) );
        
        (*this) << pose_iterator.get() << LHLV_iterator.get() << vertical_LIDAR.get() << horizontal_LIDAR.get() << pose_windower.get() << swathe_builder.get() << engine.get() << predictor.get() ;

        current.reset( new L3::SE3( L3::SE3::ZERO() ) );
 
        // This, should be TMP
        this->provider = oracle;
  
        // Timing statistics
        timings.resize(5);
   
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
            
        L3::Timing::ChronoTimer performance_timer;

        int performance_index;
           
        bool init = false;

        while( running )
        {
            performance_index = 0;
            performance_timer.begin();

            current_time += ( t.elapsed() - real_time_elapsed )*speedup;
                
            real_time_elapsed = t.elapsed(); 

            /*
             *  Update all watchers
             */
            TemporalRunner::update( current_time );
            timings[ performance_index++ ] = performance_timer.elapsed();

            static int counter = 0;
            
            if( !init )
            {
                *current = oracle->operator()();

                counter++;

                if ( counter == 60 )
                    init = true;
            }

            /*
             *  Recompute swathe
             */
            swathe_builder->update( current_time );
            timings[ performance_index++ ] = performance_timer.elapsed();

            /*
             *  Point cloud generation, projection
             */
            L3::WriteLock point_cloud_lock( projector->cloud->mutex );
            projector->project( swathe_builder->swathe );
            point_cloud_lock.unlock();
            timings[ performance_index++ ] = performance_timer.elapsed();

            
            /*
             *  Update everything else
             */
            update( current_time );
            timings[ performance_index++ ] = performance_timer.elapsed();

            if( stand_alone )
            {

                if( !output.is_open() )
                    output.open( "poses.dat", std::ios::out );

                std::stringstream ss;
                ss << "Observer update:"  << "\t" << timings[0] << std::endl;
                ss << "Swathe update:"    << "\t\t" << timings[1] << std::endl;
                ss << "Point generation:" << "\t" << timings[2] << std::endl;
                ss << "Estimation:"       << "\t\t" << timings[3] << std::endl;
                std::cout << ss.str() << "------------" << std::endl;
           
                output << *current << '\n';
            }

            /*
             * Perform post-update, don't care about how long this takes
             */
            std::for_each( updaters.begin(), updaters.end(), std::mem_fun( &Updater::update ) );
        }
   
        thread.join();
    }
   
    bool EstimatorRunner::update( double time )
    {
        /*
         *  Update the experience
         */
        experience->update( current->X(), current->Y() );

        /*
         *  Estimate
         */
        //L3::ReadLock algo_lock( this->mutex ); 
        *current = algorithm->operator()( projector->cloud, *current );
        //algo_lock.unlock();

        return true;
    }
}

