#include "Filter.h"

typedef std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > VELOCITY_WINDOW;
typedef std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > >::iterator VELOCITY_WINDOW_ITERATOR;

namespace L3
{
    namespace Estimator
    {

        struct WeightAccumulator : std::binary_function<double,double,double>
        {

            double operator()( double weight_val, double val )
            {
            }

        };

    template <typename T>
        SE3 ParticleFilter<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
        {
            boost::shared_ptr< L3::ConstantTimeIterator<L3::LHLV> > constant_time_iterator = this->iterator.lock();

            if( !constant_time_iterator  )
            {
                std::cerr << "No velocity window, cannot continue..." << std::endl;
                exit(-1);
            }

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

                initialised = true;
                return estimate;
            }

            L3::WriteLock master( this->mutex );

            // Find the new data, between the last update time and now
            // Do we need to lock this?
            L3::Iterator<L3::LHLV>::WINDOW_ITERATOR index = std::lower_bound( constant_time_iterator->window.begin(), 
                    constant_time_iterator->window.end(), 
                    previous_time,
                    comparator );
     
            VELOCITY_WINDOW _window_delta_buffer;

            // Add in the newly-seen data
            _window_delta_buffer.assign( index, constant_time_iterator->window.end() );

            if ( _window_delta_buffer.empty() )
                return estimate;
            
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

            std::vector<double> results( this->num_particles ); 
            std::vector<double>::iterator result_iterator = results.begin();

            int pyramid_index = 0;

            L3::ReadLock histogram_lock( (*this->pyramid)[pyramid_index]->mutex );
            L3::ReadLock swathe_lock( swathe->mutex );

            if( !L3::sample( swathe, this->sampled_swathe.get(), 2*1000, false ) )  
                return estimate;    // Point cloud is empty

            double q, x_vel, y_vel, velocity_delta, x, y;
            
            //boost::normal_distribution<> linear_velocity_plant_uncertainty(0.0, .5 );
            //boost::normal_distribution<> rotational_velocity_plant_uncertainty(0.0, .05 );
            
            boost::normal_distribution<> linear_velocity_plant_uncertainty(0.0, .5 );
            boost::normal_distribution<> rotational_velocity_plant_uncertainty(0.0, .05 );
            
            boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > linear_velocity_uncertainty_generator(rng, linear_velocity_plant_uncertainty );
            boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > rotational_velocity_uncertainty_generator(rng, rotational_velocity_plant_uncertainty );
           
            double dt = current_time - previous_time;
                
            previous_time = current_time;

            if ( dt > 1 )
                return estimate;
            
            std::vector< L3::SE3 > delta;

            // Apply the constant average velocities to the particles
            for( PARTICLE_ITERATOR it = hypotheses.begin();
                        it != hypotheses.end();
                        it++ )
            {
                q = it->Q() + ((-1*mean_rotational_velocity) + rotational_velocity_uncertainty_generator())*dt;
            
                velocity_delta = mean_linear_velocity + linear_velocity_uncertainty_generator() ;

                x_vel = (velocity_delta * sin(q));  
                y_vel = (velocity_delta * cos(q));
         
                x = it->X() + -1*x_vel*dt;
                y = it->Y() + y_vel*dt;

                delta.push_back( L3::SE3( x, y, 0, 0, 0, q ) );

                group.run( Hypothesis( this->sampled_swathe.get(), &*it, (*this->pyramid)[pyramid_index].get(), this->cost_function.get(), result_iterator++ ) );
            }

            group.wait();
            
            histogram_lock.unlock();
            swathe_lock.unlock();
            
            hypotheses.swap( delta );
        
            std::vector< double > _weights;
            _weights.assign( results.begin(), results.end() );
            
            double sum = std::accumulate( results.begin(), results.end(), 0.0 );

            if ( std::isnan(sum) )
            {
                std::cerr << "NAN" << std::endl;
                exit(-1);
            }

            // Normalize
            weights.resize( _weights.size() );
            std::transform( _weights.begin(), _weights.end(), weights.begin(), std::bind2nd( std::divides<double>(), sum ) );

            // Build CDF 
            std::vector< double > cdf( weights.size() );
            std::partial_sum( weights.begin(), weights.end(), cdf.begin() );

            boost::uniform_01<> uniform;

            std::vector< L3::SE3 > resampled;
            resampled.reserve( hypotheses.size() );

            // Sample
            for( int i=0; i<num_particles; i++ )
            {
                double d = uniform( rng );
                
                resampled.push_back( hypotheses[ std::distance( cdf.begin(), std::lower_bound( cdf.begin(), cdf.end(), d ) )] ); 
            }

            hypotheses.assign( resampled.begin(), resampled.end() );

            // Have to use inner product
            //L3::SE3 predicted = std::accumulate( resampled.begin(), resampled.end(), L3::SE3::ZERO() );
            //L3::SE3 predicted = std::inner_product( resampled.begin(), resampled.end(), L3::SE3::ZERO() );

            this->current_prediction = predicted/(resampled.size());

            return this->current_prediction;
        }

    template <typename T>
        bool ParticleFilter<T>::update( double time )
        {
            current_time = time;
        }
     


    }
}

template L3::SE3 L3::Estimator::ParticleFilter<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template bool L3::Estimator::ParticleFilter<double>::update(double);
