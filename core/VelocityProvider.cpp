#include  "VelocityProvider.h"
#include "Utils.h"

// Smoothing
#include "itpp/signal/filter.h"
#include "itpp/signal/freq_filt.h"
#include "itpp/signal/transforms.h"

//10-20
//itpp::vec filter = "0.000091 0.000190 0.000327 0.000505 0.000727 0.000993 0.001301 0.001646 0.002021 0.002416 0.002822 0.003224 0.003610 0.003965 0.004277 0.004534 0.004725 0.004843 0.004883 0.004843 0.004725 0.004534 0.004277 0.003965 0.003610 0.003224 0.002822 0.002416 0.002021 0.001646 0.001301 0.000993 0.000727 0.000505 0.000327 0.000190 0.000091";
 //itpp::vec filter = "0.000131 0.001215 0.002201 -0.000000 -0.005505 -0.008035 -0.002673 0.003664 -0.000000 -0.006502    0.008502    0.046832    0.060955    -0.000000    -0.107747    -0.151990    -0.056011    0.114721    0.200000    0.114721    -0.056011    -0.151990    -0.107747    -0.000000    0.060955    0.046832    0.008502    -0.006502    -0.000000    0.003664    -0.002673    -0.008035    -0.005505    -0.000000    0.002201    0.001215    0.000131 ";
 //itpp::vec filter = ".333333333  .333333333 .333333333";
 itpp::vec filter = ".1 .1 .1 .1 .1 .1 .1 .1 .1 .1"; 

namespace L3
{

    bool ScanMatchingVelocityProvider::update( double time )
    {
        if( engine->trajectory.empty() )
            return false;
        
        raw_velocities.clear();

        // Copy the window
        std::deque< std::pair< double, Eigen::Matrix4f > > trajectory;
        
        L3::ReadLock lock( engine->mutex );
        if( engine->trajectory.empty() )
            return false;
        trajectory.assign( engine->trajectory.begin(), engine->trajectory.end() );
        lock.unlock();

        std::deque< std::pair< double, Eigen::Matrix4f > >::iterator it = trajectory.begin();
        std::advance( it, 1 );

        std::vector<double> data(4);
        
        for( ; 
                it != trajectory.end();
                it++ )
        {
            // Compute instantaneous velocity
            double distance = (sqrt( pow( it->second( 0,3 ) - (it-1)->second(0,3), 2 ) 
                        + pow( it->second( 1,3 ) - (it-1)->second(1,3), 2 ) ));

            double dt = ( it->first - (it-1)->first );
            
            double velocity = distance/dt;

            if( std::isinf( velocity ) || std::isnan( velocity ) )
                continue;

            L3::SE3 previous = L3::Utils::Math::poseFromRotation( (it-1)->second );
            L3::SE3 current = L3::Utils::Math::poseFromRotation( it->second );
            double rotational_velocity = (previous.Q()-current.Q())/dt;

            data[0] = velocity;
            data[3] = rotational_velocity;

            raw_velocities.push_back( std::make_pair( it->first, data ) );
        }

        // Filter
        filtered_velocities.assign( raw_velocities.begin(), raw_velocities.end() );

        itpp::vec b;
        b.set_size(raw_velocities.size());
        for( int i=0; i<raw_velocities.size(); i++ )
            b[i] = raw_velocities[i].second[0];

        itpp::Freq_Filt<double> FF(filter,1000);

        itpp::vec res = FF.filter( b );

        for( int i=0; i<raw_velocities.size(); i++ )
            filtered_velocities[i].second[0] = res[i];
        
        return true;
    }

    struct zipper : std::unary_function< std::pair< double, boost::shared_ptr< L3::LHLV > >, std::pair< double, std::vector< double > > >
    {
        zipper()
        {
            data.resize(4);
        }
            
        std::vector< double > data;
        
        std::pair< double, std::vector< double > > operator()( std::pair< double, boost::shared_ptr< L3::LHLV > > input )     
        {
            data[0] = input.second->data[9];
            data[3] = input.second->data[3];

            return std::make_pair( input.first, data );
        }

    };

    bool LHLVVelocityProvider::update( double time  )
    {
        filtered_velocities.clear();

        zipper z;

        std::transform( iterator->window.begin(),
                iterator->window.end(),
                std::back_inserter( filtered_velocities ),
                z );

        return ( !filtered_velocities.empty() );
    }
}
