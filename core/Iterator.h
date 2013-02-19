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
    
        Iterator( L3::SlidingWindow<T>* w ) : windower(w)
        {

        }

        virtual size_t size() = 0;
        
        typename std::deque< std::pair< double, boost::shared_ptr<T> > > window;
        typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator WINDOW_ITERATOR;
        
        typename std::deque< std::pair< double, boost::shared_ptr<T> > > buffered_window;
        typedef typename std::deque< std::pair< double, boost::shared_ptr<T> > >::iterator BUFFERED_WINDOW_ITERATOR;

    protected:

        L3::SlidingWindow<T>* windower;
};

template <typename T>
struct Comparator
{
    bool operator()( T t, const double f )
    {
        return ( t.first < f);
    }
};

template <typename T>
class ConstantTimeIterator : public Iterator<T>
{
    public:

        ConstantTimeIterator( L3::SlidingWindow<T>* window, double time ) 
            : Iterator<T>( window ), swathe_length(time)
        {
            initialise();
        }

        void update( double time )
        {
            // Update the watcher with the new time
            this->windower->update( time );

            // Retrive the window
            this->buffered_window = this->windower->getWindow();

            Comparator<std::pair< double, boost::shared_ptr<T> > > c;

            // Find the element with the closest time to *now*
            typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it = std::lower_bound( this->buffered_window.begin(), this->buffered_window.end(), time, c );

            if ( it == this->buffered_window.end() ) // This, is bad - can't find the appropriate time
                throw std::exception();

            // Clear it - this needs to be pop & delete
            this->window.clear();

            double data_swathe_length = 0;

            typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it_back_iterator = it;

            std::cout.precision( 12 );

            // Working backwards, build up the data swathe
            while( data_swathe_length < swathe_length )
            {
                this->window.push_front( *it_back_iterator );
                data_swathe_length = (*it).first - (*it_back_iterator).first ;
                it_back_iterator--;
          
                // At the beginning?
                if ( it_back_iterator == this->buffered_window.begin() )
                    break; 
            }
        }

        size_t size()
        {
            return this->buffered_window.size();
        }

        double relativeTime() 
        {
            //return (*head)->time - dataset->poses[0]->time;
        }

    protected:
   
        std::string LIDAR_name;
        double swathe_length;
        
        void initialise()
        {
            //std::vector<L3::Pose*>::iterator it = dataset->poses.begin();

            //// Set tail
            //tail = it;

            //// Get first pose & associated scan
            //L3::Pose*   root_pose = dataset->poses[0];
            //L3::LMS151* root_scan = dataset->getScanAtTime( root_pose->time, LIDAR_name );

            //swathe.push_back( std::make_pair( root_pose, root_scan ) );

            //L3::Tools::Timer t;

            //t.begin();

            //double duration;
            //while( it!= dataset->poses.end() )
            //{
                //duration = (*it)->time - root_pose->time; 

                //if ( duration <  swathe_length )
                //{
                    //swathe.push_back( std::make_pair( (*it), dataset->getScanAtTime( (*it)->time, LIDAR_name ) ) );
                //}
                //else
                    //break;

                //it++;
            //}

            //// Set head
            //head = it; 
        }

        
};


} // L3

#endif
