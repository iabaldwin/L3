#include "PoseWindower.h"

namespace L3
{
    typedef std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*> SWATHE_ITERATOR;

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
        std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > _window_delta_buffer;

        // Add in the newly-seen data
        _window_delta_buffer.assign( index, constant_time_iterator->window.end() );

        // Append it to the buffer
        _window_buffer.insert( _window_buffer.end(), _window_delta_buffer.begin(), _window_delta_buffer.end() );

        _constant_distance_window.resize( _window_buffer.size()) ;

        int written;
        double total_distance;
        SWATHE_ITERATOR iterator = L3::reverseTrajectoryAccumulate( _window_buffer.rbegin(),
                _window_buffer.rend(),
                _constant_distance_window.begin(),
                0.2, 
                std::numeric_limits<double>::infinity(), 
                written);

        //_window_buffer.erase( _window_buffer.begin(), _window_buffer.end() - written  );
        _window_buffer.erase( _window_buffer.begin(), _window_buffer.begin() + written );
        _constant_distance_window.erase( iterator, _constant_distance_window.end() );

        // Update
        previous_update = time;

        return true;
    }

}

template bool L3::Inverter::invert<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*> >(std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*>);
