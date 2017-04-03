#pragma once

#include "Iterator.h"
#include "Integrator.h"
#include "VelocityProvider.h"
#include "Datatypes.h"

namespace L3
{

  class Predictor : public L3::TemporalObserver
  {
    public:

      Predictor( L3::VelocityProvider* provider ) 
        : provider(provider),
        previous_update(0.0)
    {
    }

      bool update( double t );

      bool predict( const L3::SE3& current );

    protected:

      L3::VelocityProvider*      provider;

      double previous_update;

      std::deque< double > sink;
  };

}
