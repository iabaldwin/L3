#ifndef L3_ITERATOR_H
#define L3_ITERATOR_H

#include <iostream>
#include <fstream>
#include <list>

#include "Tools.h"
#include "Dataset.h"
#include "Datatypes.h"

namespace L3
{

class Iterator
{

    public:
    
        Iterator( L3::Dataset* DATASET ) : dataset(DATASET)
        {
        }

        virtual void update( double dt ) = 0;

    protected:

        std::list< std::pair< L3::Pose*, L3::LMS151* > > swathe; 

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

        void update( dt )
        {


        }

    protected:

        void initialise()
        {
            std::vector<L3::Pose*>::iterator it = dataset->poses.begin();

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

        }

        std::string LIDAR_name;
        double swathe_length;

};


} // L3

#endif 
