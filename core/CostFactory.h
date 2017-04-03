#pragma once

#include "Estimator.h"

namespace L3
{
namespace Estimator
{
  template <typename T>
  struct CostFactory {
    static boost::shared_ptr< CostFunction<T> > produce(std::string cost_function)
    {
      if(cost_function == "MI") {
        return boost::make_shared< MICostFunction<T> >();
      }

      if(cost_function == "NMI") {
        return boost::make_shared< NMICostFunction<T> >();
      }

      if(cost_function == "KL") {
        return boost::make_shared< KLCostFunction<T> >();
      }

      if(cost_function == "SSD") {
        return boost::make_shared< SSDCostFunction<T> >();
      }

      return boost::shared_ptr< CostFunction<T> >(); 
    }
  };

} // namespace Estimator
} // namespace L3
