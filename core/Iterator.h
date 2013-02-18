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

namespace L3
{

template <typename T>
class Iterator : public Observer
{
    public:
    
        Iterator( L3::SlidingWindow<T>* w ) : windower(w)
        {

        }

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
            current_window = this->windower->getWindow();

            Comparator<std::pair< double, std::string > > c;

            // Find the element with the closest time to *now*
            WINDOW_ITERATOR it = std::lower_bound( current_window.begin(), current_window.end(), time, c );

            if ( it == current_window.end() ) // This, is bad
                throw std::exception();

            this->processed_data.clear();

            double data_swathe_length = 0;

            // Working backwards, build up the data swathe
            while( data_swathe_length < swathe_length )
            {
                //this->processed_data.push_front( *it );
                data_swathe_length += current_window.back().first - (*it).first;
            }

        }

        typename std::deque< std::pair< double, T*> > processed_data;

        int numScans()
        {
            //return std::distance( tail, head ); 
        }

        double relativeTime() 
        {
            //return (*head)->time - dataset->poses[0]->time;
        }

    protected:

        WINDOW current_window;

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

        std::string LIDAR_name;
        double swathe_length;
};


} // L3

#endif
