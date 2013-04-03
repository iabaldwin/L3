#ifndef L3_ITERATOR_H
#define L3_ITERATOR_H

#include <iostream>
#include <fstream>
#include <list>

#include "Core.h"
#include "Datatypes.h"
#include "Definitions.h"
#include "Windower.h"
#include "Tools.h"
#include "AbstractFactory.h"
#include "PoseProvider.h"

namespace L3
{

template <typename T>
class Iterator : public TemporalObserver
{
    public:
    
        Iterator( boost::shared_ptr<L3::SlidingWindow<T> > w ) : windower(w)
        {
        }

        typename std::deque< std::pair< double, boost::shared_ptr<T> > > window;
        typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator WINDOW_ITERATOR;

        void getWindow( typename std::deque< std::pair< double, boost::shared_ptr<T> > >& window);

    protected:

        Poco::Mutex mutex;

        typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator BUFFERED_WINDOW_ITERATOR;
        typename std::deque< std::pair< double, boost::shared_ptr<T> > > buffered_window;

        boost::shared_ptr< L3::SlidingWindow<T> > windower;
};

template <typename T>
class ConstantTimeIterator : public Iterator<T>
{
    public:

        ConstantTimeIterator( boost::shared_ptr< L3::SlidingWindow<T> > window )
            : Iterator<T>( window ), 
                swathe_length(window->window_duration)
        {
        }

        // Swathe length, in seconds
        double swathe_length;
        
        // <time, pose> comparison
        Comparator<std::pair< double, boost::shared_ptr<T> > > _pair_comparator;
       
        // Core update
        bool update( double time );
};

class ConstantTimePoseWindower : public PoseWindower
{
    public:
        
        ConstantTimePoseWindower( L3::ConstantTimeIterator<L3::SE3>* iterator ) 
            : constant_time_iterator (iterator)
        {
            this->window = &(iterator->window);
        }

        bool update( double t)
        {
            constant_time_iterator->update(t);
        }

        L3::ConstantTimeIterator<L3::SE3>* constant_time_iterator;

        L3::SE3 operator()( void )
        {

            if ( this->constant_time_iterator->window.size() > 0 )
                return *(this->constant_time_iterator->window.back().second);
            else
                return L3::SE3::ZERO();
        }
};

} // L3

#endif
