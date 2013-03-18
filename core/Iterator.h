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

    protected:
        
        typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator BUFFERED_WINDOW_ITERATOR;
        typename std::deque< std::pair< double, boost::shared_ptr<T> > > buffered_window;

        boost::shared_ptr< L3::SlidingWindow<T> > windower;
};

template <typename T>
class ConstantTimeIterator : public Iterator<T>
{
    public:

        ConstantTimeIterator( boost::shared_ptr< L3::SlidingWindow<T> > window, double time ) 
            : Iterator<T>( window ), swathe_length(time)
        {
        }

        double swathe_length;
        Comparator<std::pair< double, boost::shared_ptr<T> > > _pair_comparator;
        
        bool update( double time );
};


} // L3

#endif
