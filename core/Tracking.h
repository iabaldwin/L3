#pragma once

#include <iostream>
#include "Core.h"

namespace L3
{
  namespace Tracking
  {
    struct AlphaBetaFilter
    {

      struct state
      {
        double x,v; 

        state& operator=( const state& lhs )
        {
          this->x = lhs.x;
          this->v = lhs.v;

          return *this;
        }


      } _state;

      AlphaBetaFilter( float alpha, float beta, double x=0.0, double v=0.0) 
        : alpha(alpha), beta(beta),
        previous_update(0.0)
      {
        _state.x = x;
        _state.v = v;
      }

      float alpha, beta;
      double previous_update;

      void update( const double time, const double measurement );


    };


    std::ostream& operator<<( std::ostream& out, const AlphaBetaFilter::state& state );

    struct KalmanFilter : Updateable
    {
      KalmanFilter()
      {

      }

      virtual void update( ) = 0;
    };

    struct EKF
    {




    };

  }

}
