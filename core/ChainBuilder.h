#ifndef L3_CHAIN_BUILDER_H
#define L3_CHAIN_BUILDER_H

#include <cmath>
#include <numeric>
#include <iterator>

#include "Datatypes.h"
#include "Definitions.h"

namespace L3
{

typedef std::pair< double, boost::shared_ptr<L3::LHLV> > RECORD;

std::ostream& operator<<( std::ostream& o, const RECORD& r ) 
{
    o<< r.first << " ";

    r.second->print(o);

    return o;
}

template <typename InputIterator, typename OutputIterator >
void trajectoryAccumulate( InputIterator begin, InputIterator end, OutputIterator output )
{
    typedef typename std::iterator_traits<OutputIterator>::value_type VAR;

    InputIterator current_in = begin;

    // Compute current time
    double current_time = current_in++->first;

    // Initialise
    double dt = 0.0;
    
    *output++ = std::make_pair( current_time, boost::shared_ptr<L3::SE3>( new L3::SE3( 0,0,0,0,0,0 ) ) );

    double x, y, z;
    double r, p, q;
    
    double w1, w2, w3;
    double lin_vel, x_vel, y_vel, z_vel;
    
    boost::shared_ptr< L3::SE3 > previous_pose;
    
    while ( current_in != end )
    {
        // Compute the update
        dt = current_in->first - current_time;
        current_time = current_in->first;

        // Compute roll, pitch, yaw
        //w1 = current_in->second->LHLV_iterator.window[5];
        w1 = current_in->second->data[5];
        //w2 = current_in->second->LHLV_iterator.window[4];
        w2 = current_in->second->data[4];
        //w3 = current_in->second->LHLV_iterator.window[3];
        w3 = current_in->second->data[3];

        previous_pose = boost::dynamic_pointer_cast< L3::SE3 >( (*(output-1)).second );

        r = previous_pose->r + w1*dt;
        p = previous_pose->p + w2*dt;
        q = previous_pose->q + w3*dt;
    
        // Compute x_bar, y_bar, z_bar 
        lin_vel = current_in->second->data[9];
      
        x_vel = lin_vel * sin(q);  
        y_vel = lin_vel * cos(q);  
        z_vel = lin_vel * sin(p);  
              
        // Compute x y z
        x = previous_pose->x + x_vel*dt;
        y = previous_pose->y + y_vel*dt;
        z = previous_pose->z + z_vel*dt;
        
        // Log
        *output++ = std::make_pair( current_time, boost::shared_ptr<L3::SE3>( new L3::SE3( x, y, z, r, p , q ) )); 
       
        // Continue
        current_in++;
    }

}

class ChainBuilder : public Observer
{
    public:

        ChainBuilder( L3::Iterator<L3::LHLV>* iterator ) : LHLV_iterator(iterator)
        {

        }

        bool update( double time )
        {

            if (!LHLV_iterator->update( time ))
                throw std::exception();

            // Reset
            window.clear();

            // Allocate
            window.resize( LHLV_iterator->window.size() );

            // Accumulate 
            L3::trajectoryAccumulate( LHLV_iterator->window.begin(), 
                                        LHLV_iterator->window.end(), 
                                        window.begin() );

            return true;
        }
            
        POSE_SEQUENCE window;

    private:

            L3::Iterator<L3::LHLV>*      LHLV_iterator;

};

}

#endif
