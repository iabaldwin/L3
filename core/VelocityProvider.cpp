#include  "VelocityProvider.h"
#include "Utils.h"

typedef std::deque <std::pair< double, Eigen::Matrix4f > >::iterator TRAJECTORY_ITERATOR;
typedef std::deque <std::pair< double, boost::shared_ptr< L3::LHLV > > >::iterator LHLV_ITERATOR;

namespace L3
{

    ScanMatchingVelocityProvider::ScanMatchingVelocityProvider( L3::ScanMatching::Engine* engine  ) 
        : engine(engine)
    {
    }


    bool ScanMatchingVelocityProvider::update( double time )
    {
        L3::WriteLock master( this->mutex );
        
        raw_velocities.push_back(  engine->raw_velocity_data );
        filtered_velocities.push_back(  engine->filtered_velocity_data );

        if( raw_velocities.back().first - raw_velocities.front().first > 10.0 )
            raw_velocities.pop_front();

        if( filtered_velocities.back().first - filtered_velocities.front().first > 10.0 )
            filtered_velocities.pop_front();

        master.unlock();

        return true;
    }

    FilteredScanMatchingVelocityProvider::FilteredScanMatchingVelocityProvider( boost::shared_ptr< L3::ConstantTimeIterator< L3::SMVelocity > > velocity_provider ) 
        : velocity_provider(velocity_provider)
    {
        _linear_velocity_filter = boost::make_shared< L3::Tracking::AlphaBetaFilter >(.1, .01);
        _rotational_velocity_filter = boost::make_shared< L3::Tracking::AlphaBetaFilter >(.5,0.1);
    
        raw_velocity_data.second.resize( 4);
        filtered_velocity_data.second.resize( 4);
    }

    struct sm_zipper : std::unary_function< std::pair<double, boost::shared_ptr<SMVelocity> >, std::pair< double, std::vector<double> > >
    {
    
        sm_zipper()
        {
            data.resize(4);
        }

        std::vector<double> data;

        std::pair< double, std::vector<double> > operator()( std::pair<double, boost::shared_ptr<SMVelocity> > entry )
        {
            return std::make_pair( entry.first, entry.second->data );
        }
    };

    bool FilteredScanMatchingVelocityProvider::update( double time )
    {
        boost::shared_ptr< L3::ConstantTimeIterator< L3::SMVelocity > > velocity_provider_ptr = velocity_provider.lock(); 

        if( !velocity_provider_ptr || velocity_provider_ptr->window.empty()  )
            return false;

        L3::WriteLock master( this->mutex );
        L3::ReadLock  velocity_lock( velocity_provider_ptr->mutex );

        double linear_velocity     = velocity_provider_ptr->window.back().second->data[0];
        double rotational_velocity = velocity_provider_ptr->window.back().second->data[3];
      
        if ( rotational_velocity < -1.6 || rotational_velocity > 1.6 )
            rotational_velocity = 0.0;

        _linear_velocity_filter->update( velocity_provider_ptr->window.back().first, linear_velocity );
        _rotational_velocity_filter->update( velocity_provider_ptr->window.back().first, rotational_velocity );

        raw_velocity_data.first = velocity_provider_ptr->window.back().first;
        raw_velocity_data.second[0] = linear_velocity;
        raw_velocity_data.second[3] = rotational_velocity;

        filtered_velocity_data.first = velocity_provider_ptr->window.back().first;
        filtered_velocity_data.second[0] = _linear_velocity_filter->_state.x;
        filtered_velocity_data.second[3] = rotational_velocity;

        velocity_lock.unlock();

        if( rotational_velocity > 1.4 || rotational_velocity < -1.4 )
        {
            std::cout << rotational_velocity << std::endl;
            exit( -1 );
        }

        filtered_velocity_data.second[0] *= scaling_bias;

        // Log
        raw_velocities.push_back( raw_velocity_data );
        filtered_velocities.push_back( filtered_velocity_data );

        // Cull
        while( filtered_velocities.back().first - filtered_velocities.front().first > 10.0 )
                filtered_velocities.pop_front();

        while( raw_velocities.back().first - raw_velocities.front().first > 10.0 )
                raw_velocities.pop_front();
  
        master.unlock();
        
        return true;
    };
    
    bool LHLVVelocityProvider::update( double time  )
    {
        L3::WriteLock master( this->mutex );
        
        filtered_velocities.clear();

        std::transform( iterator->window.begin(), 
                iterator->window.end(),
                std::back_inserter( filtered_velocities ),
                z );

        master.unlock();

        return ( !filtered_velocities.empty() );
    }
}
