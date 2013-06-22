#include "Runner.h"

#include <boost/make_shared.hpp>

namespace L3
{

    DatasetRunner::DatasetRunner( L3::Dataset* dataset, L3::Configuration::Mission* mission, float speedup ) 
        : mission(mission),
            dataset(dataset), 
            speedup(speedup),
            current_time(0.0),
            start_time(dataset->start_time),
            stand_alone(false)
    {
        // Constant time iterator over poses
        pose_iterator = boost::make_shared< L3::ConstantTimeIterator<L3::SE3> >( dataset->pose_reader );
        //LHLV_iterator = boost::make_shared< L3::ConstantTimeIterator<L3::LHLV> >( dataset->LHLV_reader );  
       
        // LIDAR iterators
        horizontal_LIDAR = boost::make_shared< L3::ConstantTimeIterator<L3::LMS151> >( dataset->LIDAR_readers[ mission->horizontal] );
        vertical_LIDAR   = boost::make_shared< L3::ConstantTimeIterator<L3::LMS151> >( dataset->LIDAR_readers[ mission->declined] );
        
        // Velocity iterator 
        velocity_source = boost::make_shared< L3::ConstantTimeIterator< L3::SMVelocity > >( dataset->velocity_reader );

        // Point-clouds
        point_cloud = boost::make_shared< L3::PointCloud<double> >();
        projection = boost::make_shared< L3::SE3 >();

        // Get the calibration
        L3::Configuration::convert( mission->lidars[mission->declined], *projection );
       
        // Point projector
        projector = boost::make_shared< L3::Projector<double> >( projection.get(), point_cloud.get() );
      
        // Scan matching engine
        engine = boost::make_shared< L3::ScanMatching::Engine >( horizontal_LIDAR.get() );

        // Velocity providers
        //lhlv_velocity_provider = boost::make_shared< L3::LHLVVelocityProvider>( LHLV_iterator.get() );
        icp_velocity_provider  = boost::make_shared< L3::ScanMatchingVelocityProvider >( engine.get() );
        ics_velocity_provider  = boost::make_shared< L3::FilteredScanMatchingVelocityProvider>( velocity_source );

        // Pose Provider
        //pose_windower = boost::make_shared< L3::ConstantTimeWindower < L3::LHLV> > ( LHLV_iterator.get() );
        //pose_windower = boost::make_shared< L3::ConstantDistanceWindower > ( lhlv_velocity_provider.get(), 20 );
        pose_windower = boost::make_shared< L3::ConstantDistanceWindower > ( ics_velocity_provider.get(), 50 );
       
        // Swathe generator
        swathe_builder = boost::make_shared< L3::RawSwatheBuilder>( pose_windower.get(), vertical_LIDAR.get() );
        //swathe_builder = boost::make_shared< L3::BufferedSwatheBuilder >( pose_windower.get(), vertical_LIDAR.get() );
   
        // INS pose
        oracle = boost::make_shared< L3::ConstantTimeWindower< L3::SE3 > > ( pose_iterator.get() );
        
        // Predictor
        //predictor = boost::make_shared< L3::Predictor >( LHLV_iterator.get() );
        
        //  Zero-th pose
        current = boost::make_shared< L3::SE3 >();
        
        // Timing container
        timings.resize(5);
      
        /*
         *Note: order is important here
         */
        (*this) << pose_iterator.get() 
                //<< LHLV_iterator.get() 
                << vertical_LIDAR.get() 
                << horizontal_LIDAR.get() 
                //<< lhlv_velocity_provider.get()
                << ics_velocity_provider.get() 
                << icp_velocity_provider.get() 
                << velocity_source.get()
                << engine.get() 
                << pose_windower.get() 
                << swathe_builder.get();
                //<< predictor.get();
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
            
        L3::Timing::SysTimer performance_timer;

        int performance_index;
           
        while( running )
        {
            if( !paused )
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

                /*
                 *Recompute swathe
                 */
                swathe_builder->update( 0 );
                timings[ performance_index++ ] = performance_timer.elapsed();

                /*
                 *Point cloud generation, projection
                 */
                L3::WriteLock point_cloud_lock( projector->cloud->mutex );
                projector->project( swathe_builder->swathe );
                point_cloud_lock.unlock();
                timings[ performance_index++ ] = performance_timer.elapsed();


                /*
                 *Update everything else
                 */
                update( 0 );
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
            else
                usleep( .5*1e6 );
  
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
         *  Do estimation
         */

        static int counter = 0;

        if( counter++ < 1000 )
            *current = oracle->operator()();
        else
        {
            L3::ReadLock algo_lock( this->mutex ); 
            *current = algorithm->operator()( projector->cloud, *current );
        }

        return true;
    }
}

