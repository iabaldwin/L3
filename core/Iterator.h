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
struct Extractor
{

    T operator()( std::string& raw )
    {

    }


};


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
        //return ( t.first < f);
        return ( f < t.first );
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

            // Find the iterator with the closest time
            WINDOW_ITERATOR it = std::lower_bound( current_window.begin(), current_window.end(), time, c );

            std::cout.precision( 12 );

            std::cout << time << "-->" << current_window.front().first << ":" << current_window.back().first << std::endl;

            //if ( it != current_window.end() )
                //std::cout << (*it).first << ":" << (*it).first - time << std::endl;

            //double previous_head_time = (*head)->time;

            //// Advance the head pointer by dt
            //while( (*head)->time - previous_head_time < dt )
            //{
                //swathe.push_back( std::make_pair( (*head), dataset->getScanAtTime( (*head)->time, LIDAR_name ) ) ) ;
                //head++;

                //if ( head == dataset->poses.end() )
                    //return false;
            //}

            //// Advance the tail, such that we preserve swathe length
            //while( ((*head)->time - (*tail)->time ) > swathe_length )
            //{
                //swathe.pop_back();
                //tail++;
            //}

#ifndef NDEBUG
            //std::cout << (*head)->time - (*tail)->time << std::endl;
#endif

        }

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
