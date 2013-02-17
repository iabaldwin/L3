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
    
        Iterator( L3::SlidingWindow<T>* w ) : window(w)
        {

        }

    protected:

        L3::SlidingWindow<T>* window;
        //std::vector< std::pair< double, T*> > sequence;
        //typename std::vector< std::pair< double, T*> >::iterator head;
        //typename std::vector< std::pair< double, T*> >::iterator tail;

};

template <typename T>
class ConstantTimeIterator : public Iterator<T>
{
    public:

        ConstantTimeIterator( L3::SlidingWindow<T>* window, double time ) 
            : Iterator<T>( window ), 
                swathe_length(time)
        {
            initialise();
        }

        void update( double dt )
        {
            //if( head == dataset->poses.end() )
                //return false;

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
