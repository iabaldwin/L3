#pragma once

#include "Components.h"
#include "L3.h"

namespace L3
{
  namespace Visualisers
  {
    struct VisualiserRunner : L3::Visualisers::Leaf, L3::TemporalRunner
    {
      VisualiserRunner( double start_time=0.0 )
        : time(start_time)
      {
      }

      double time;

      void onDraw3D( glv::GLV& g )
      {
        L3::TemporalRunner::update( time += 1.0/10.0 );
        std::cout.precision( 16 );
        std::cout << time << std::endl;
      }

      VisualiserRunner& operator<<( L3::TemporalObserver* observer)
      {
        if ( observer ) {
          observers.push_front( observer ); 
        }
        return *this;
      }
    };
  } // Visualisers
} // L3
