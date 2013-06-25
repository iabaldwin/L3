#ifndef L3_ITERATOR_H
#define L3_ITERATOR_H

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
                throw std::runtime_error( "Invalid window" ) ;
        }

        virtual ~Iterator()
        {
        }
        
        Poco::Mutex m;


        typename std::deque< std::pair< double, boost::shared_ptr<T> > > window;
        typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator WINDOW_ITERATOR;

        //void getWindow( typename std::deque< std::pair< double, boost::shared_ptr<T> > >& window);

    protected:

        typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator BUFFERED_WINDOW_ITERATOR;
        typename std::deque< std::pair< double, boost::shared_ptr<T> > > buffered_window;

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

#endif
