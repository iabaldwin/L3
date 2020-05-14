#pragma once

#include "Datatypes.h"
#include "PoseProvider.h"
#include "VelocityProvider.h"

namespace L3
{
  /*
   *  Pose Windower
   */
  struct PoseWindower : PoseProvider, TemporalObserver
  {
    std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > >* window;
  };

  /*
   *  Constant time INS windower
   */
  struct Inverter
  {
    std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > chain;

    template <typename InputIterator >
      bool invert( InputIterator start, InputIterator end );
  };


  template <typename T>
    class ConstantTimeWindower : public PoseWindower, Lockable
  {
    public:

      ConstantTimeWindower( L3::ConstantTimeIterator<T>* iterator ) 
        : constant_time_iterator (iterator)
      {
        this->window = &(iterator->window);
      }

      Inverter inverter;
      L3::ConstantTimeIterator<T>* constant_time_iterator;

      bool update( double t)
      {
        L3::WriteLock lock( this->mutex );
        return constant_time_iterator->update(t);
      }

      L3::SE3 operator()( void )
      {
        L3::ReadLock lock( this->mutex );

        if ( !this->constant_time_iterator->window.empty() )
          return *(this->constant_time_iterator->window.back().second);
        else
          return L3::SE3::ZERO();
      }
  };

  /*
   *  Constant distance windowers
   */
  class ConstantDistanceWindower : public PoseWindower
  {
    public:

      ConstantDistanceWindower( L3::VelocityProvider* provider, double swathe_length = 10.0  ) 
        : velocity_provider(provider),
        swathe_length(swathe_length),
        previous_update(0.0)
    {
      // Reflection
      this->window = &_constant_distance_window;
    }

      // Provider base
      L3::VelocityProvider* velocity_provider;

      // Core window
      std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > _constant_distance_window;

      // Window buffer
      VELOCITY_WINDOW _window_buffer;

      // Search structure
      Comparator< std::pair< double, std::vector< double > > > comparator;

      double swathe_length;

      double previous_update;

      bool update(double time) {
        if(velocity_provider->filtered_velocities.empty()) {
          return false;
        }

        // Find the new data, between the last update time and now
        L3::VelocityProvider::VELOCITY_ITERATOR index = std::lower_bound(velocity_provider->filtered_velocities.begin(),
            velocity_provider->filtered_velocities.end(),
            previous_update,
            comparator);

        if (index->first == previous_update) {
          index++;
        }

        // Delta buffer
        VELOCITY_WINDOW _window_delta_buffer;
        _window_delta_buffer.assign(index, velocity_provider->filtered_velocities.end());

        if (_window_delta_buffer.empty()) {
          return false;
        }

        // Append it to the buffer
        _window_buffer.insert(_window_buffer.end(), _window_delta_buffer.begin(), _window_delta_buffer.end());
        _constant_distance_window.clear();

        int written;

        L3::reverseTrajectoryAccumulate(_window_buffer.rbegin(),
            _window_buffer.rend(),
            std::back_inserter(_constant_distance_window),
            0.2, 
            swathe_length,
            written);

        std::reverse(_constant_distance_window.begin(), _constant_distance_window.end());

        _window_buffer.erase(_window_buffer.begin(), _window_buffer.begin() + (_window_buffer.size() - written));

        // Update
        previous_update = _window_buffer.back().first;

        return true;
      }
  };
} // L3

#include "PoseWindower.hpp"
