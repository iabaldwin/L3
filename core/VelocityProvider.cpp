#include  "VelocityProvider.h"
#include "Utils.h"

// Smoothing
#include "itpp/signal/filter.h"
#include "itpp/signal/freq_filt.h"
#include "itpp/signal/transforms.h"

namespace L3
{

    bool ScanMatchingVelocityProvider::update( double time )
    {
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

        for( ; it != trajectory.end();
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

            std::vector<double> data(4);
            data[0] = velocity;
            data[3] = rotational_velocity;

            raw_velocities.push_back( std::make_pair( it->first, data ) );
        }

        //itpp::vec filter = itpp::fir1 (int N, double cutoff)
        itpp::vec filter = itpp::fir1 (50,.01);

        itpp::vec b;
        b.set_size(raw_velocities.size());
        for( int i=0; i<raw_velocities.size(); i++ )
            b[i] = raw_velocities[i].second[0];

        itpp::Freq_Filt<double> FF(filter,1000);

        itpp::vec res = FF.filter( b );

        filtered_velocities.assign( raw_velocities.begin(), raw_velocities.end() );

        std::vector<double> data(4);
       
        for( int i=0; i<raw_velocities.size(); i++ )
            filtered_velocities[i].second[0] = res[i];


        // Compute fft

        if ( b.size() > 0 )
        {
        itpp::cvec fft_res;

        fft_res  = itpp::fft_real(b);

        //std::cout << fft_res << std::endl;

        for( int i = 0; i<fft_res.size() ; i++ )
            std::cout << fft_res[i] << std::endl;

        }
        std::cout << "--------------" << std::endl;
        return true;
    }

    struct zipper : std::unary_function< std::pair< double, boost::shared_ptr< L3::LHLV > >, std::pair< double, std::vector< double > > >
    {
        std::pair< double, std::vector< double > > operator()( std::pair< double, boost::shared_ptr< L3::LHLV > > input )     
        {
            std::vector< double > data( 4 );
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
