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

            if ( !initialised && !(estimate == L3::SE3::ZERO() ) )
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
                    *it = L3::SE3( x_generator()+estimate.X(), y_generator()+estimate.Y(), 0, 0, 0, theta_generator()+estimate.Q() );

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

            double dt = 0.1;

            std::vector<double> results( this->num_particles ); 
            std::vector<double>::iterator result_iterator = results.begin();

            L3::ReadLock histogram_lock( (*this->pyramid)[0]->mutex );
            L3::ReadLock swathe_lock( swathe->mutex );

            if( !L3::sample( swathe, this->sampled_swathe.get(), 2*1000, false ) )
                return estimate;
            
            // Apply the constant average velocities to the particles
            for( PARTICLE_ITERATOR it = hypotheses.begin();
                        it != hypotheses.end();
                        it++ )
            {

                double q = it->Q() + (-1*mean_rotational_velocity)*dt;
                double x_vel = mean_linear_velocity * sin(q);  
                double y_vel = mean_linear_velocity * cos(q);  
         
                double x = it->X() + -1*x_vel*dt;
                double y = it->Y() + y_vel*dt;

                *it = L3::SE3( x, y, 0, 0, 0, q );

                group.run( Hypothesis( this->sampled_swathe.get(), &*it, (*this->pyramid)[0].get(), this->cost_function, result_iterator++ ) );
            }

            group.wait();

            histogram_lock.unlock();
            swathe_lock.unlock();

            weights.assign( results.begin(), results.end() );
            
            double sum = std::accumulate( results.begin(), results.end(), 0.0 );

            if ( std::isnan(sum) )
                return estimate;

            // Normalize
            std::transform( weights.begin(), weights.end(), weights.begin(), std::bind2nd( std::divides<double>(), sum ) );

            return estimate;
        }

    }
}

template L3::SE3 L3::Estimator::ParticleFilter<double>::operator()(L3::PointCloud<double>*, L3::SE3);
