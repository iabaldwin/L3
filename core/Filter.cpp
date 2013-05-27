#include "Filter.h"

typedef std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > VELOCITY_WINDOW;
typedef std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > >::iterator VELOCITY_WINDOW_ITERATOR;

namespace L3
{
    namespace Estimator
    {
 
    template <typename T>
        SE3 ParticleFilter<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
        {
            boost::shared_ptr< L3::ConstantTimeIterator<L3::LHLV> > constant_time_iterator = this->iterator.lock();

            if( !constant_time_iterator  )
                exit(-1);

            if ( !initialised )
            {

                // Generate the initial spread of particles
                boost::normal_distribution<> normal_x(0.0, 1.0 );
                boost::normal_distribution<> normal_y(0.0, 1.0 );
                boost::normal_distribution<> normal_theta(0.0, 0.1 );
            
                boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > x_generator(rng, normal_x );
                boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > y_generator(rng, normal_y );
                boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > theta_generator(rng, normal_theta  );

                for( PARTICLE_ITERATOR it = hypotheses.begin();
                        it != hypotheses.end();
                        it++ )
                {
                    *it = L3::SE3( x_generator()+estimate.X(), y_generator()+estimate.Y(), 0, 0, 0, theta_generator()+estimate.Q() );
                }

                // But do not evaluate their cost

                initialised = true;
                return estimate;
            }

            // Find the new data, between the last update time and now
            L3::Iterator<L3::LHLV>::WINDOW_ITERATOR index = std::lower_bound( constant_time_iterator->window.begin(), 
                    constant_time_iterator->window.end(), 
                    previous_update,
                    comparator );
        
            VELOCITY_WINDOW _window_delta_buffer;

            // Add in the newly-seen data
            _window_delta_buffer.assign( index, constant_time_iterator->window.end() );

            double mean_linear_velocity = 0.0;
            double mean_rotational_velocity = 0.0;

            // Calculate average rotational and linear velocity
            for( VELOCITY_WINDOW_ITERATOR it = _window_delta_buffer.begin();
                    it!= _window_delta_buffer.end();
                    it++ )
            {
                mean_linear_velocity += it->second->data[9];
                mean_rotational_velocity += it->second->data[3];
            }

            // Compute the average velocities
            mean_linear_velocity /= _window_delta_buffer.size();
            mean_rotational_velocity /= _window_delta_buffer.size();

            // Apply the constant average velocities to the particles


            return estimate;
        }

    }
}

template L3::SE3 L3::Estimator::ParticleFilter<double>::operator()(L3::PointCloud<double>*, L3::SE3);
