#pragma once

#include <list>

#include "boost/shared_ptr.hpp"
#include "boost/make_shared.hpp"
#include "boost/exception/all.hpp"
#include "boost/thread/shared_mutex.hpp"
#include "glog/logging.h"

namespace L3
{
  typedef boost::shared_lock<boost::shared_mutex> ReadLock;
  typedef boost::unique_lock<boost::shared_mutex> WriteLock;

  struct Lockable {
    boost::shared_mutex mutex;
  };

  struct Observer {
    std::string name{"unknown"};
  };

  struct Updateable {
    virtual void update() = 0;
  };

  struct Updater : Lockable {
    std::list < Updateable* > updateables;

    void update();

    Updater& operator<<(Updateable* updateable) {
      CHECK_NOTNULL(updateable);
      L3::WriteLock lock(this->mutex);
      updateables.push_front(updateable);
      lock.unlock();
      return *this;
    }

    void remove(Updateable* updateable);
  };

  struct TemporalObserver : Observer {
    virtual bool update(double) = 0;
  };

  struct SpatialObserver : Observer {
    virtual bool update(double, double) = 0;
  };

  /*
   *Exceptions
   */
  struct exception_base: virtual std::exception, virtual boost::exception { };

  struct end_of_stream: virtual exception_base { };
  struct LIDAR_end: virtual end_of_stream { };
  struct POSE_end: virtual end_of_stream { };
  struct lookup_failure: virtual exception_base { };
  struct calibration_failure : virtual exception_base { };
  struct no_such_file : virtual exception_base { };
  struct no_such_folder : virtual exception_base { };

  template <typename T>
    struct Comparator
    {
      bool operator()(const T t, const double f) const
      {
        return (t.first < f);
      }
    };

} // L3
