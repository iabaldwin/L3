#include "Runner.h"

#include <boost/make_shared.hpp>

namespace L3
{
  DatasetRunner::DatasetRunner(L3::Dataset* dataset, L3::Configuration::Mission* mission, float speedup) 
    : mission(mission),
    dataset(dataset), 
    speedup(speedup),
    current_time(0.0),
    start_time(dataset->start_time),
    stand_alone(false),
    booted(false),
    run_mode(RunMode::Continuous),
    frequency(0.0) {
      // Iterator over poses
#ifndef NDEBUG
      std::cout << __FILE__ << ":" << "Assigning pose iterator" << std::endl;
#endif
      pose_iterator = boost::make_shared< L3::ConstantTimeIterator<L3::SE3> >(dataset->pose_reader);

      // Iterator over velocity
#ifndef NDEBUG
      std::cout << __FILE__ << ":" << "Assigning velocity source" << std::endl;
#endif
      LHLV_iterator = boost::make_shared< L3::ConstantTimeIterator<L3::LHLV> >(dataset->LHLV_reader);  

      // LIDAR iterators
#ifndef NDEBUG
      std::cout << __FILE__ << ":" << "Assigning LIDAR iterators" << std::endl;
      std::cout << mission->horizontal << std::endl;
      std::cout << dataset->LIDAR_readers[mission->horizontal] << std::endl;
#endif
      assert(dataset->LIDAR_readers[ mission->horizontal]);
      assert(dataset->LIDAR_readers[ mission->declined]);
      horizontal_LIDAR = boost::make_shared< L3::ConstantTimeIterator<L3::LMS151> >(dataset->LIDAR_readers[ mission->horizontal]);
      vertical_LIDAR   = boost::make_shared< L3::ConstantTimeIterator<L3::LMS151> >(dataset->LIDAR_readers[ mission->declined]);

      // Scan-match velocity source
#ifndef NDEBUG
      std::cout << __FILE__ << ":" << "Assigning scan-match iterator " << std::endl;
#endif
      velocity_source = boost::make_shared< L3::ConstantTimeIterator< L3::SMVelocity > >(dataset->velocity_reader);

      // Point-clouds
      point_cloud = boost::make_shared< L3::PointCloud<double> >();
      horizontal_projection = boost::make_shared< L3::SE3 >();
      vertical_projection = boost::make_shared< L3::SE3 >();

      // Get the calibration
      L3::Configuration::convert(mission->lidars[mission->declined], *vertical_projection);
      L3::Configuration::convert(mission->lidars[mission->horizontal], *horizontal_projection);

      // Point projector
      projector = boost::make_shared< L3::Projector<double> >(vertical_projection.get(), point_cloud.get());

      // Scan matching engine
      engine = boost::make_shared< L3::ScanMatching::Engine >(horizontal_LIDAR.get());

      // Velocity providers
      lhlv_velocity_provider = boost::make_shared< L3::LHLVVelocityProvider >(LHLV_iterator.get());                 // INS
      icp_velocity_provider  = boost::make_shared< L3::ScanMatchingVelocityProvider >(engine.get());                // ICP
      ics_velocity_provider  = boost::make_shared< L3::FilteredScanMatchingVelocityProvider>(velocity_source);      // ICS

      // Pose Provider
      pose_windower = boost::make_shared< L3::ConstantDistanceWindower > (ics_velocity_provider.get(), 30);

      // Swathe generator
      swathe_builder = boost::make_shared< L3::BufferedSwatheBuilder >(pose_windower.get(), vertical_LIDAR.get());

      // INS pose
      oracle = boost::make_shared< L3::ConstantTimeWindower< L3::SE3 > > (pose_iterator.get());

      // Predictor
      predictor = boost::make_shared< L3::Predictor >(ics_velocity_provider.get());

      //  Zero-th pose
      estimated_pose = boost::make_shared< L3::SE3 >();
      oracle_pose = boost::make_shared< L3::SE3 >();

      // Timing container
      timings.resize(5);

      /*
       *  Note: order is important here
       */
      (*this) << pose_iterator.get() 
        << velocity_source.get()
        << LHLV_iterator.get() 
        << vertical_LIDAR.get() 
        << horizontal_LIDAR.get() 
        << lhlv_velocity_provider.get()
        << ics_velocity_provider.get() 
        << icp_velocity_provider.get() 
        << engine.get() 
        << pose_windower.get() 
        << swathe_builder.get()
        << predictor.get();
    }

  /*
   *  Dataset runner
   */
  void DatasetRunner::run() {
    L3::Timing::ChronoTimer system_timer;

    system_timer.begin();

    double real_time_elapsed = system_timer.elapsed();

    current_time = start_time;

    L3::Timing::SysTimer performance_timer, frequency_timer;

    int performance_index;

    int counts = 0;

    //int boot = 200;
    int boot = 800;

    if(stand_alone) {
      boot = 2000;
      openStreams();
    }

    int total_index = 0;

    while(running) {
      if(!paused) {
        frequency_timer.begin();
        performance_index = 0;
        performance_timer.begin();

        switch(run_mode)
        {
          case RunMode::Continuous:
            // Get the delta
            current_time += (system_timer.elapsed() - real_time_elapsed)*speedup;
            real_time_elapsed = system_timer.elapsed(); 
            break;

          case RunMode::Step: 
            current_time += .1;
            paused = true; 
            break;

          default:
            throw std::exception();
        }

        /*
         *  Update all watchers
         */
        performance_timer.begin();
        TemporalRunner::update(current_time);
        timings[ performance_index++ ] = performance_timer.elapsed();

        /*
         *  Recompute swathe
         */
        performance_timer.begin();
        swathe_builder->update(0);
        timings[ performance_index++ ] = performance_timer.elapsed();

        /*
         *  Point cloud generation, projection
         */
        performance_timer.begin();
        projector->project(swathe_builder->swathe);
        timings[ performance_index++ ] = performance_timer.elapsed();

        /*
         *  Update everything else
         */
        performance_timer.begin();
        update(0);
        timings[ performance_index++ ] = performance_timer.elapsed();

        if(stand_alone) {
          std::stringstream ss;

          ss << std::setprecision(5)  ;
          ss << timings[0] << "," <<
            timings[1] << "," <<
            timings[2] << "," <<
            timings[3];

          statistics_output << ss.str() << std::endl;

          ss.str(std::string());

          pose_output << *estimated_pose << '\n';     // Buffer

          total_index++;

          if (total_index > 6000)
            exit(0);
        } else {
          double _frequency = 1.0/frequency_timer.elapsed();

          if (_frequency < 80.0) {
            frequency = (_frequency + counts*frequency)/(++counts);
          }
        }

        /* 
         * Perform post-update, don't care about how long this takes. In headless mode,
         * there wouldn't be anything here except for a publisher. 
         */
        std::for_each(updaters.begin(), updaters.end(), std::mem_fun(&Updater::update));

        boot--;

        if(boot == 0)
          booted = true;
      } else {
        usleep(.1*1e6);
        real_time_elapsed = system_timer.elapsed();
      }
    }
    thread.join();
  }

  bool EstimatorRunner::update(double time) {
    /*
     *  Update the experience
     */
    experience->update(estimated_pose->X(), estimated_pose->Y());

    /*
     *  Do estimation
     */
    static int counter = 0;

    *oracle_pose = oracle->operator()();

    /*
     *  Boot
     */
    if(!booted) {
      *estimated_pose = experience->getClosestPose(oracle->operator()());
      estimated_pose->Q(oracle_pose->Q()); 
      //*estimated_pose = *oracle_pose;
    } else {
      L3::ReadLock algo_lock(this->mutex); 
      *estimated_pose = algorithm->operator()(projector->cloud, *estimated_pose);
    }
    return true;
  }
}
