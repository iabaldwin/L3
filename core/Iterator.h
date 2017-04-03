#pragma once

#include <iostream>
#include <fstream>
#include <list>

#include "Core.h"
#include "Datatypes.h"
#include "Definitions.h"
#include "Windower.h"
#include "Timing.h"

namespace L3
{

  template <typename T>
    class Iterator : public TemporalObserver, public Lockable
  {
    public:

      Iterator( boost::shared_ptr<L3::SlidingWindow<T> > window ) : windower(window)
    {
      if ( !window )
        throw std::runtime_error( std::string( __FILE__ ) +"Invalid window" ) ;
    }

      virtual ~Iterator()
      {
      }

      std::mutex m;

      typename std::deque< std::pair< double, boost::shared_ptr<T> > > window;
      typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator WINDOW_ITERATOR;

    protected:

      typename std::deque< std::pair< double, boost::shared_ptr<T> > > buffered_window;
      typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator BUFFERED_WINDOW_ITERATOR;

      boost::weak_ptr< L3::SlidingWindow<T> > windower;
  };

  template <typename T>
    class ConstantTimeIterator : public Iterator<T>
  {
    public:

      ConstantTimeIterator( boost::shared_ptr< L3::SlidingWindow<T> > window, double duration=10.0 )
        : Iterator<T>( window ), 
        swathe_length(duration)
    {
    }

      virtual ~ConstantTimeIterator()
      {
      }

      // Swathe length, in seconds
      double swathe_length;

      // <time, pose> comparison
      Comparator<std::pair< double, boost::shared_ptr<T> > > _pair_comparator;

      // Core update
      bool update( double time );
  };

} // L3
