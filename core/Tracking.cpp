#include "Tracking.h"

namespace L3
{
  namespace Tracking
  {
    void AlphaBetaFilter::update(const double time, const double measurement) {
      /*
       *Predict
       */
      double dt = time - previous_update;

      if(dt > 1 || dt == 0)  {
        previous_update = time;
        return;
      }

      double _alpha = this->alpha;
      double _beta  = this->beta;

      state _current_state;

      _current_state.x = _state.x + dt *_state.v;
      _current_state.v = _state.v;

      /*
       *Update
       */
      double residual = measurement - _current_state.x ;

      _current_state.x = _current_state.x  + _alpha*residual;
      _current_state.v = _current_state.v  + (_beta/dt)*residual;

      _state = _current_state;

      previous_update = time;
    }

    std::ostream& operator<<(std::ostream& out, const AlphaBetaFilter::state& state) {
      out << "State: " << state.x << ','  << state.v;
      return out;
    }
  } // Tracking
} // L3
