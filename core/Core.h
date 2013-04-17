#ifndef L3_CORE_H
#define L3_CORE_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/exception/all.hpp>
#include <boost/thread/shared_mutex.hpp>

namespace L3
{

struct Observer
{

    virtual ~Observer()
    {

    }

};


struct TemporalObserver : Observer
{
    virtual bool update( double ) = 0;
};

struct SpatialObserver : Observer
{
    virtual bool update( double, double ) = 0;
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
    bool operator()( T t, const double f )
    {
        return ( t.first < f);
    }

    bool operator()( const double f, T t )
    {
        return ( t.first < f);
    }

};

typedef boost::shared_mutex Mutex;
typedef boost::shared_lock<Mutex> ReadLock;
typedef boost::unique_lock<Mutex> WriteLock;

struct Lockable
{
    boost::shared_mutex mutex;
};


}

#endif
