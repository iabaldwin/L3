#include "Runner.h"

#include "boost/make_shared.hpp"

namespace L3
{
  DatasetRunner::DatasetRunner(L3::Dataset* dataset, L3::Configuration::Mission* mission, float speedup, bool stand_alone)
    : mission(mission),
    dataset(dataset),
    speedup(speedup),
    current_time(0.0),
    start_time(dataset->start_time),
    stand_alone(stand_alone),
    booted(false),
    run_mode(RunMode::Continuous),
    frequency(0.0) {

      // Iterator over poses
      pose_iterator = boost::make_shared< L3::ConstantTimeIterator<L3::SE3> >(dataset->pose_reader);
      CHECK_NOTNULL(pose_iterator);
      pose_iterator->name = "PoseIterator";

      // Iterator over velocity
      LHLV_iterator = boost::make_shared< L3::ConstantTimeIterator<L3::LHLV> >(dataset->LHLV_reader);
      LHLV_iterator->name = "LHLVIterator";
      CHECK_NOTNULL(LHLV_iterator);

      // LIDAR iterators
      LOG(INFO) << "Assigning LIDAR iterators";
      LOG(INFO) << "Horizontal: " << mission->horizontal;
      LOG(INFO) << "Declined: " << mission->declined;

      CHECK(dataset->LIDAR_readers[ mission->horizontal]);
      CHECK(dataset->LIDAR_readers[ mission->declined]);
      horizontal_LIDAR = boost::make_shared< L3::ConstantTimeIterator<L3::LMS151> >(dataset->LIDAR_readers[mission->horizontal]);
      horizontal_LIDAR->name = "HorizontalLidar";
      CHECK_NOTNULL(horizontal_LIDAR);
      vertical_LIDAR = boost::make_shared< L3::ConstantTimeIterator<L3::LMS151> >(dataset->LIDAR_readers[mission->declined]);
      CHECK_NOTNULL(vertical_LIDAR);
      vertical_LIDAR->name = "VerticalLidar";

      // Scan-match velocity source
      velocity_source = boost::make_shared< L3::ConstantTimeIterator< L3::SMVelocity > >(dataset->velocity_reader);
      CHECK_NOTNULL(velocity_source);
      velocity_source->name = "SMVelocity";

      // Point-clouds
      point_cloud = boost::make_shared< L3::PointCloud<double> >();
      horizontal_projection = boost::make_shared< L3::SE3 >();
      vertical_projection = boost::make_shared< L3::SE3 >();

      // Get the calibration
      L3::Configuration::convert(mission->lidars[mission->declined], *vertical_projection);
      L3::Configuration::convert(mission->lidars[mission->horizontal], *horizontal_projection);

      // Point projector
      projector = boost::make_shared< L3::Projector<double> >(vertical_projection.get(), point_cloud.get());
      CHECK_NOTNULL(projector);

      // Scan matching engine
      engine = boost::make_shared< L3::ScanMatching::Engine >(horizontal_LIDAR.get());
      CHECK_NOTNULL(engine);

      // Velocity providers
      lhlv_velocity_provider = boost::make_shared< L3::LHLVVelocityProvider >(LHLV_iterator.get());                 // INS
      CHECK_NOTNULL(lhlv_velocity_provider);
      lhlv_velocity_provider->name = "LHLVVelocity";

      icp_velocity_provider  = boost::make_shared< L3::ScanMatchingVelocityProvider >(engine.get());                // ICP
      CHECK_NOTNULL(icp_velocity_provider);
      icp_velocity_provider->name = "ICPVelocity";

      ics_velocity_provider  = boost::make_shared< L3::FilteredScanMatchingVelocityProvider>(velocity_source);      // ICS
      CHECK_NOTNULL(ics_velocity_provider);
      ics_velocity_provider->name = "ICSVelocity";

      // Pose Provider
      pose_windower = boost::make_shared< L3::ConstantDistanceWindower > (ics_velocity_provider.get(), 30);
      CHECK_NOTNULL(pose_windower);
      pose_windower->name = "PoseWindower";

      // Swathe generator
      swathe_builder = boost::make_shared< L3::BufferedSwatheBuilder >(pose_windower.get(), vertical_LIDAR.get());
      CHECK_NOTNULL(swathe_builder);
      swathe_builder->name = "SwatheBuilder";

      // INS pose
      oracle = boost::make_shared< L3::ConstantTimeWindower< L3::SE3 > > (pose_iterator.get());
      CHECK_NOTNULL(oracle);

      // Predictor
      predictor = boost::make_shared< L3::Predictor >(ics_velocity_provider.get());
      CHECK_NOTNULL(predictor);
      predictor->name = "Predictor";

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

    int boot = 800;
    if(stand_alone) {
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

          pose_output << *estimated_pose << '\n';     // Buffer

          total_index++;
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

        if(boot == 0) {
          booted = true;
        }
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
    *oracle_pose = oracle->operator()();

    /*
     *  Boot
     */
    if(not booted) {
      *estimated_pose = experience->getClosestPose(oracle->operator()());
      estimated_pose->Q(oracle_pose->Q());
    } else {
      L3::ReadLock algo_lock(this->mutex);
      *estimated_pose = algorithm->operator()(projector->cloud, *estimated_pose);
      if (stand_alone) {
        LOG_EVERY_N(INFO, 50) << *estimated_pose;
      }
    }
    return true;
  }
} // L3
