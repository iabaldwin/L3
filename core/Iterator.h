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
class Iterator : public Observer
{
    public:
    
        Iterator( boost::shared_ptr<L3::SlidingWindow<T> > w ) : windower(w)
        {
        }

        typename std::deque< std::pair< double, boost::shared_ptr<T> > > window;
        typename std::deque< std::pair< double, boost::shared_ptr<T> > > buffered_window;
        
        typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator WINDOW_ITERATOR;
        typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator BUFFERED_WINDOW_ITERATOR;

    protected:

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

        Comparator<std::pair< double, boost::shared_ptr<T> > > _pair_comparator;
        
        bool update( double time )
        {
            // Update the watcher with the new time
            this->windower->update( time );

            // Retrive the buffered window
            this->buffered_window = this->windower->getWindow();

            // Find the element with the closest time to *now*
            typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it = std::lower_bound( this->buffered_window.begin(), this->buffered_window.end(), time, _pair_comparator );

            if ( it == this->buffered_window.end() ) // This, is bad - can't find the appropriate time
            {
                std::cout.precision(15);
                std::cout << __PRETTY_FUNCTION__ << time << "->" << this->buffered_window.front().first << ":" << this->buffered_window.back().first << std::endl;
                return false; 
            }

            it = ( fabs(( it-1 )->first - time)  ) < ( fabs(( it )->first - time)  ) ? ( it-1) : it ;

            // Got it?
            this->window.clear();

            double data_swathe_length = 0;

            typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it_back_iterator = it;

            // Working backwards, build up the data swathe
            while( data_swathe_length < swathe_length )
            {
                // At the beginning? Is this all we have?
                if ( it_back_iterator == this->buffered_window.begin() )
                    break; 

                this->window.push_front( *it_back_iterator );

                // Compute dt
                data_swathe_length = (*it).first - (*it_back_iterator).first ;
               
                // Continue
                it_back_iterator--;

            }

            return true;
        }

    //protected:
   
        double swathe_length;
};


} // L3

#endif
