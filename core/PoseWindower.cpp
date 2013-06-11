#include "PoseWindower.h"
#include <iterator> 
#include <deque> 

namespace L3
{
    template <typename InputIterator >
        bool Inverter::invert(InputIterator start, InputIterator end )
        {
            if ( std::distance( start, end ) == 0 )
                return false;

            // Destructive resize 
            chain.assign( start, end );

            // End point
            Eigen::Matrix4f last_pose = chain.back().second->getHomogeneous(); 

            Eigen::Matrix4f delta( last_pose.inverse()  );

            // Transform
            std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = chain.begin();

            while( it != chain.end() )
            {
                Eigen::Matrix4f tmp( it->second->getHomogeneous() ); 

                it->second->setHomogeneous( delta * tmp );

                it++;
            }

            return true;
        }

    bool ConstantDistanceWindower::update( double time )
    {
        // Find the new data, between the last update time and now
        L3::VelocityProvider::VELOCITY_ITERATOR index = std::lower_bound( velocity_provider->filtered_velocities.begin(),
                velocity_provider->filtered_velocities.end(),
                previous_update,
                comparator );

        // Create a delta buffer
        VELOCITY_WINDOW _window_delta_buffer;
        _window_delta_buffer.assign( index, velocity_provider->filtered_velocities.end() );
        
        if (_window_delta_buffer.empty() )
            return false;

        // Append it to the buffer
        _window_buffer.insert( _window_buffer.end(), _window_delta_buffer.begin(), _window_delta_buffer.end() );
        _constant_distance_window.clear();

        int written;
        double total_distance;

        L3::reverseTrajectoryAccumulate( _window_buffer.rbegin(),
                _window_buffer.rend(),
                std::back_inserter( _constant_distance_window ),
                0.1, 
                swathe_length,
                written);

        std::reverse( _constant_distance_window.begin(), _constant_distance_window.end() );
    
        _window_buffer.erase( _window_buffer.begin(), _window_buffer.begin() + ( _window_buffer.size() - written ) );

        // Update
        previous_update = time;

        return true;
    }

}

template bool L3::Inverter::invert<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*> >(std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*>);
