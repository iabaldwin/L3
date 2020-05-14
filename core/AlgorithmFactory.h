#pragma once

#include "Runner.h"
#include "Filter.h"
#include "Estimator.h"

namespace L3
{
namespace Estimator
{

template <typename T>
struct AlgorithmFactory
{
  static boost::shared_ptr< Algorithm<T> > produce(std::string algorithm,
      boost::shared_ptr< CostFunction<T> > cost_function = boost::shared_ptr< CostFunction<T> >(),
      boost::shared_ptr< L3::HistogramPyramid<T> > pyramid = boost::shared_ptr<L3::HistogramPyramid<T> >(),
      boost::shared_ptr< L3::EstimatorRunner > runner = boost::shared_ptr< L3::EstimatorRunner >())
  {
    if(algorithm == "ID") {
      return boost::make_shared< IterativeDescent<T> >(cost_function, pyramid);
    }

    if(algorithm == "Min") {
      return boost::make_shared< Minimisation<T> >(cost_function, pyramid);
    }

    if(algorithm == "Hybrid") {
      return boost::make_shared< Hybrid<T> >(cost_function, pyramid);
    }

    if(algorithm == "PF" && runner) {
      if(boost::shared_ptr< L3::ConstantDistanceWindower > windower  = boost::dynamic_pointer_cast< L3::ConstantDistanceWindower >(runner->pose_windower))
        windower->swathe_length = 25.0;

      if(boost::shared_ptr< L3::HistogramUniformDistance <double> > hist_ptr  =
          boost::dynamic_pointer_cast< L3::HistogramUniformDistance<double> >((*(runner->experience->experience_pyramid))[0]))
      {
        hist_ptr->bins_per_metre = .85;
      }

      runner->experience->window = 4;

      return boost::make_shared< ParticleFilter<T> >(cost_function, pyramid, runner->ics_velocity_provider);
    }

    if(algorithm == "UKF" && runner) {
      return boost::make_shared< UKF<T> >(cost_function, pyramid, runner->ics_velocity_provider);
    }
    return boost::shared_ptr< Algorithm<T> >();
  }
};

} // Estimator
} // L3
