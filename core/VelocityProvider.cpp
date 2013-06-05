#include  "VelocityProvider.h"

namespace L3
{

    bool ScanMatchingVelocityProvider::update( double time )
    {
        velocities.clear();

        // Copy the window

        std::deque< std::pair< double, Eigen::Matrix4f > > trajectory;
        
        L3::ReadLock lock( engine->mutex );
        if( engine->trajectory.empty() )
            return false;
        trajectory.assign( engine->trajectory.begin(), engine->trajectory.end() );
        lock.unlock();

        for( std::deque< std::pair< double, Eigen::Matrix4f > >::iterator it = (trajectory.begin()+1);
                it != trajectory.end();
                it++ )
        {
            // Compute instantaneous velocity
            //double instantaneous_velocity = (sqrt( pow( it->second( 0,3 ) - (it-1)->second(0,3), 2 ) 
                        //+ pow( it->second( 1,3 ) - (it-1)->second(1,3), 2 ) ))/( it->first - (it-1)->first );

            double distance = (sqrt( pow( it->second( 0,3 ) - (it-1)->second(0,3), 2 ) 
                        + pow( it->second( 1,3 ) - (it-1)->second(1,3), 2 ) ));

            double dt = ( it->first - (it-1)->first );

            double velocity = distance/dt;

            if( std::isinf( velocity ) || std::isnan( velocity ) )
                continue;
            
            velocities.push_back( std::make_pair( it->first, std::make_pair( velocity, 0.0 ) ) );
        }

        return true;
    }
}
