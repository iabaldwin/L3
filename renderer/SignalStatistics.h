#ifndef L3_SIGNAL_STATISTICS_H
#define L3_SIGNAL_STATISTICS_H

#include <iostream>

#include <gsl/gsl_histogram.h>

#include <glv.h>

namespace L3
{
  namespace Visualisers
  {

    struct SignalStatistics : L3::BasicPlottable<double>
    {

      SignalStatistics()
      {

      }

    };
  }
}
#endif
