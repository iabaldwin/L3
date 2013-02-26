#ifndef L3_CORE_H
#define L3_CORE_H

#include <boost/exception/all.hpp>

namespace L3
{

struct Observer
{
    virtual bool update( double ) = 0;
};

struct exception_base: virtual std::exception, virtual boost::exception { };

struct end_of_stream: virtual exception_base { };
struct LIDAR_end: virtual end_of_stream { };
struct POSE_end: virtual end_of_stream { };

struct lookup_failure: virtual exception_base { };

}

#endif
