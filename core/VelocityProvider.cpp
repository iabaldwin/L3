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
          
            std::vector<double> data(4);
            data[0] = velocity;

            //velocities.push_back( std::make_pair( it->first, std::make_pair( velocity, 0.0 ) ) );
            velocities.push_back( std::make_pair( it->first, data ) );
        }

        return true;
    }

    struct zipper : std::unary_function< std::pair< double, boost::shared_ptr< L3::LHLV > >, std::pair< double, std::vector< double > > >
    {
        std::pair< double, std::vector< double > > operator()( std::pair< double, boost::shared_ptr< L3::LHLV > > input )     
        {
            std::vector< double > data( 4 );
            data[0] = input.second->data[9];
            data[3] = input.second->data[3];
            //return std::make_pair( input.first, std::make_pair( input.second->data[9], input.second->data[3] ) );
            return std::make_pair( input.first, data );
        }

    };

    bool LHLVVelocityProvider::update( double time  )
    {
        velocities.clear();

        zipper z;

        std::transform( iterator->window.begin(),
                iterator->window.end(),
                std::back_inserter( velocities ),
                z );

        return ( !velocities.empty() );
    }
}
