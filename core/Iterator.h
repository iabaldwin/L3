#ifndef L3_ITERATOR_H
#define L3_ITERATOR_H

#include <iostream>
#include <fstream>
#include <list>

#include "Tools.h"
#include "Dataset.h"
#include "Datatypes.h"
#include "Definitions.h"

namespace L3
{

class Iterator
{

    public:
    
        Iterator( L3::Dataset* DATASET ) : dataset(DATASET)
        {
        }

        virtual int     numScans()          = 0;
        virtual bool    update( double dt ) = 0;

        SWATHE* getSwathe()
        {
            return &swathe;
        }

    protected:

        SWATHE swathe;
        POSE_SEQUENCE::iterator head;
        POSE_SEQUENCE::iterator tail;

        L3::Dataset* dataset;

};

class ConstantTimeIterator : public Iterator
{

    public:

        ConstantTimeIterator( L3::Dataset* DATASET, const  std::string lidar_name, double time ) 
            : Iterator( DATASET ), swathe_length(time), LIDAR_name(lidar_name)
            {
                initialise();
            }

        bool update( double dt )
        {
            if( head == dataset->poses.end() )
                return false;

            double previous_head_time = (*head)->time;

            // Advance the head pointer by dt
            while( (*head)->time - previous_head_time < dt )
            {
                swathe.push_front( std::make_pair( (*head), dataset->getScanAtTime( (*head)->time, LIDAR_name ) ) ) ;
                head++;

                if ( head == dataset->poses.end() )
                    return false;
            }
          
            // Advance the tail, such that we preserve swathe length
            while( ((*head)->time - (*tail)->time ) > swathe_length )
            {
                swathe.pop_back();
                tail++;
            }

            #ifdef DEBUG
            std::cout << (*head)->time - (*tail)->time << std::endl;
            #endif

            return true;

        }

        int numScans()
        {
            return std::distance( tail, head ); 
        }

        double relativeTime() 
        {
            return (*head)->time - dataset->poses[0]->time;
        }

    protected:


        void initialise()
        {
            std::vector<L3::Pose*>::iterator it = dataset->poses.begin();

            // Set tail
            tail = it;

            // Get first pose & associated scan
            L3::Pose*   root_pose = dataset->poses[0];
            L3::LMS151* root_scan = dataset->getScanAtTime( root_pose->time, LIDAR_name );

            swathe.push_front( std::make_pair( root_pose, root_scan ) );

            L3::Tools::Timer t;

            t.begin();

            double duration;
            while( it!= dataset->poses.end() )
            {
                duration = (*it)->time - root_pose->time; 
                
                if ( duration <  swathe_length )
                {
                    swathe.push_front( std::make_pair( (*it), dataset->getScanAtTime( (*it)->time, LIDAR_name ) ) );
                }
                else
                    break;
                
                it++;
            }

            // Set head
            head = it; 
        }

        std::string LIDAR_name;
        double swathe_length;
};


} // L3

#endif 
