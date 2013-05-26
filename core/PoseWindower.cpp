#include "PoseWindower.h"

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
        L3::Iterator<L3::LHLV>::WINDOW_ITERATOR index = std::lower_bound( constant_time_iterator->window.begin(), 
                constant_time_iterator->window.end(), 
                previous_update,
                comparator );

        // Create a delta buffer
        std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > _pose_delta_buffer;
        std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > _window_delta_buffer;
       
        // Allocate
        _window_delta_buffer.assign( index, constant_time_iterator->window.end() );
        _pose_delta_buffer.resize( _window_delta_buffer.size() );

        // Create delta incremental distance buffer
        if ( _window_delta_buffer.empty() )
            return false;

        std::deque < double > _incremental_distances( _window_delta_buffer.size() );
      
        // Compute swathe length for the delta
        double incremental_distance = L3::trajectoryAccumulate( _window_delta_buffer.begin(), 
                                                                _window_delta_buffer.end(), 
                                                                _pose_delta_buffer.begin(), 
                                                                _incremental_distances.begin() );

        double required_distance = swathe_length - incremental_distance;

        // Find a pointer to the last element, so we can pop it off
        std::deque < double >::reverse_iterator _window_incremental_distances_iterator  = _window_incremental_distances.rend();

        while( required_distance > 0 && (_window_incremental_distances_iterator != _window_incremental_distances.rbegin() ) )
        {
            double tmp_dist = *(--_window_incremental_distances_iterator);

            required_distance -= tmp_dist;
        }

        //unsigned int elements_to_remove = std::distance( _window_incremental_distances.rbegin(), _window_incremental_distances_iterator );
        unsigned int elements_to_remove = std::distance( _window_incremental_distances.rbegin(), _window_incremental_distances_iterator );

        _window_buffer.erase( _window_buffer.begin(), _window_buffer.begin() + elements_to_remove );
        _window_incremental_distances.erase( _window_incremental_distances.begin(), _window_incremental_distances.begin() + elements_to_remove );

        // Join  windows
        _window_buffer.insert( _window_buffer.end(), _window_delta_buffer.begin(), _window_delta_buffer.end() ); 
        _window_incremental_distances.insert( _window_incremental_distances.end(), _incremental_distances.begin(), _incremental_distances.end() ); 

        // Resize sink
        _sink.resize( _window_buffer.size() );

        _constant_distance_window.resize( _window_buffer.size() );

        std::cout << L3::trajectoryAccumulate( _window_buffer.begin(),
                                    _window_buffer.end(),
                                    _constant_distance_window.begin(),
                                    _sink.begin() ) << " metres" << std::endl;
        // Update
        previous_update = time;

        // Lastly, invert the chain
        return inverter.invert( _constant_distance_window.begin(), _constant_distance_window.end() );
    }

}

template bool L3::Inverter::invert<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*> >(std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*>);
