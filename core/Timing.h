#pragma once

#include <sys/time.h>
#include <ctime>
#include <boost/chrono.hpp>

namespace L3
{
  namespace Timing
  {

    struct Timer
    {

      virtual void begin() = 0;

      virtual double elapsed() = 0;

    };

    struct SysTimer : Timer
    {
      clock_t tStart;

      void begin()
      {
        tStart = clock();
      }

      double elapsed()
      {
        return (double)(clock() - tStart)/CLOCKS_PER_SEC;
      }

    };


    struct ChronoTimer : Timer
    {
      boost::chrono::system_clock::time_point start;

      void begin()
      {
        start = boost::chrono::system_clock::now();
      }

      double elapsed()
      {
        boost::chrono::duration<double> sec = boost::chrono::system_clock::now() - start;
        return sec.count();
      }
    };

  } // Timing
} // L3
