#include "Filter.h"

#include "Integrator.h"
#include "Utils.h"

namespace L3
{
    namespace Estimator
    {

        const Bayesian_filter_matrix::Vec& PredictionModel::f (const Bayesian_filter_matrix::Vec &x) const
        {

            boost::shared_ptr< L3::VelocityProvider > iterator_ptr =  iterator.lock();

            if (!iterator_ptr )
                return x;

            if ( last_update == 0 )
            {
                // Not initialised
                last_update = iterator_ptr->filtered_velocities.back().first;
                return x;
            }

            L3::VelocityProvider::VELOCITY_COMPARATOR c;

            L3::VelocityProvider::VELOCITY_ITERATOR index = std::lower_bound( iterator_ptr->filtered_velocities.begin(),
                    iterator_ptr->filtered_velocities.end(),
                    last_update,
                    c );

            std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > _window( std::distance( index, iterator_ptr->filtered_velocities.end() ) );

            // No data
            if ( _window.size() <= 1 )
                return x;

            double distance;

            trajectoryAccumulate( index, 
                    iterator_ptr->filtered_velocities.end(),
                    _window.begin(),
                    distance,
                    std::numeric_limits<double>::infinity()
                    );

            if( !_window.empty() )
                std::cout << *(_window.back().second) <<  std::endl;

            L3::SE3 previous_pose( x[0], x[1], 0, 0, 0, x[2] );

            Eigen::Matrix4f tmp(  previous_pose.getHomogeneous() );
            tmp *= _window.back().second->getHomogeneous();

            L3::SE3 prediction = L3::Utils::Math::poseFromRotation( tmp );

            Bayesian_filter_matrix::Vec estimate(3);
            estimate[0] = prediction.X();
            estimate[1] = prediction.Y();
            estimate[2] = prediction.Q();
            Fx->assign( estimate );

            // Update times
            last_update = iterator_ptr->filtered_velocities.back().first;
            
            return *Fx;
        }

        ObservationModel::ObservationModel () : Bayesian_filter::Linear_uncorrelated_observe_model(3,3)
        {
            // Linear model
            //Hx(0,0) = 1;
            // Constant Observation Noise model with variance of one
            //Zv[0] = 1.;
        }

        template <typename T>
            EKF<T>::EKF( boost::shared_ptr<CostFunction<T> > cost_function,  
                    boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, 
                    boost::shared_ptr< L3::VelocityProvider > iterator )
            : Filter<T>(iterator), 
            Algorithm<T>(cost_function),
            initialised(false)
        {
            ukf.reset( new Bayesian_filter::Unscented_scheme(3) );
      
            x_init.reset( new Bayesian_filter_matrix::Vec(3) );
            X_init.reset( new Bayesian_filter_matrix::SymMatrix(3,3) );
	
            prediction_model  = boost::make_shared< PredictionModel >(  iterator );
            observation_model = boost::make_shared< ObservationModel >();
        }

        template <typename T>
            SE3 EKF<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
            {
                if( !initialised )
                {
                    (*x_init)[0]= estimate.X();
                    (*x_init)[1]= estimate.Y();
                    (*x_init)[2]= estimate.Q();
               
                    ukf->init_kalman (*x_init, *X_init);
      
                    initialised = true;
                }

                ukf->predict( *prediction_model);
                ukf->update();
            }

        template <typename T>
            bool EKF<T>::update( double time )
            {
                if ( initialised )
                {

                    current_time = time;

                    prediction_model->update( current_time );

                    return true;
                }
                return false;
            }



        template <typename T>
            SE3 ParticleFilter<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
            {
                boost::shared_ptr< L3::VelocityProvider > velocity_provider = this->iterator.lock();

                
                int _num_particles = this->num_particles;

                double _linear_uncertainty = this->linear_uncertainty;
                double _rotational_uncertainty = this->rotational_uncertainty;

                hypotheses.resize( _num_particles );

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


                // Find the new data, between the last update time and now
                VELOCITY_WINDOW_ITERATOR index = std::lower_bound( velocity_provider->filtered_velocities.begin(), 
                        velocity_provider->filtered_velocities.end(), 
                        previous_time,
                        comparator );

                VELOCITY_WINDOW _window_delta_buffer;

                // Add in the newly-seen data
                _window_delta_buffer.assign( index, velocity_provider->filtered_velocities.end() );

                if ( _window_delta_buffer.empty() )
                    return estimate;

              
                double mean_linear_velocity = 0.0;
                double mean_rotational_velocity = 0.0;

                // Calculate average rotational and linear velocity
                for( VELOCITY_WINDOW_ITERATOR it = _window_delta_buffer.begin();
                        it!= _window_delta_buffer.end();
                        it++ )
                {
                    mean_linear_velocity += it->second[0];
                    mean_rotational_velocity += it->second[3];
                }

                // Compute the average velocities
                mean_linear_velocity /= _window_delta_buffer.size();
                mean_rotational_velocity /= _window_delta_buffer.size();

                std::vector<double> results( _num_particles ); 
                std::vector<double>::iterator result_iterator = results.begin();

                int pyramid_index = 0;

                L3::ReadLock histogram_lock( (*this->pyramid)[pyramid_index]->mutex );

                double q, x_vel, y_vel, velocity_delta, x, y;

                //boost::normal_distribution<> linear_velocity_plant_uncertainty(0.0, 3 );
                //boost::normal_distribution<> rotational_velocity_plant_uncertainty(0.0, .2 );

                boost::normal_distribution<> linear_velocity_plant_uncertainty(0.0, _linear_uncertainty);
                boost::normal_distribution<> rotational_velocity_plant_uncertainty(0.0, _rotational_uncertainty );


                boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > linear_velocity_uncertainty_generator(rng, linear_velocity_plant_uncertainty );
                boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > rotational_velocity_uncertainty_generator(rng, rotational_velocity_plant_uncertainty );

                // Compute time elapsed
                double dt = current_time - previous_time;

                std::vector< L3::SE3 > delta;
                delta.reserve( hypotheses.size() );

                previous_time = current_time;

                if ( dt > 1 )
                    return estimate;

                if( !L3::sample( swathe, this->sampled_swathe.get(), 2*1000, false ) )  
                    throw std::exception();

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
                    //Hypothesis( this->sampled_swathe.get(), &*it, (*this->pyramid)[pyramid_index].get(), this->cost_function.get(), result_iterator++ )();
                }

                group.wait();

                histogram_lock.unlock();

                hypotheses.swap( delta );

                std::vector< double > _weights;
                _weights.assign( results.begin(), results.end() );

                double sum = std::accumulate( results.begin(), results.end(), 0.0 );

                // Normalize
                weights.resize( _weights.size() );
                std::transform( _weights.begin(), _weights.end(), weights.begin(), std::bind2nd( std::divides<double>(), sum ) );

                this->current_prediction = std::inner_product( hypotheses.begin(), hypotheses.end(), weights.begin(), L3::SE3::ZERO() );

                // Build CDF 
                std::vector< double > cdf( weights.size() );
                std::partial_sum( weights.begin(), weights.end(), cdf.begin() );

                boost::uniform_01<> uniform;
                std::vector< L3::SE3 > resampled;
                resampled.reserve( hypotheses.size() );

                //Sample
                for( int i=0; i<_num_particles; i++ )
                {
                    double d = uniform( rng );

                    resampled.push_back( hypotheses[ std::distance( cdf.begin(), std::lower_bound( cdf.begin(), cdf.end(), d ) )] ); 
                }

                hypotheses.swap( resampled );
                
                return this->current_prediction;
           
            }

        template <typename T>
            bool ParticleFilter<T>::update( double time )
            {
                current_time = time;

                return true;
            }



    }

    L3::SE3 operator+( const L3::SE3& lhs,  const L3::SE3& rhs )
    {
        L3::SE3 retval;
        retval.X( lhs.X() + rhs.X() );
        retval.Y( lhs.Y() + rhs.Y() );
        retval.Z( lhs.Z() + rhs.Z() );
        retval.R( lhs.R() + rhs.R() );
        retval.P( lhs.P() + rhs.P() );
        retval.Q( lhs.Q() + rhs.Q() );

        return retval;
    }

    L3::SE3 operator/( const L3::SE3& lhs,  const double divisor )
    {
        L3::SE3 retval( lhs );

        retval.X( lhs.X()/divisor );
        retval.Y( lhs.Y()/divisor );
        retval.Z( lhs.Z()/divisor );
        retval.R( lhs.R()/divisor );
        retval.P( lhs.P()/divisor );
        retval.Q( lhs.Q()/divisor );

        return retval;
    }

    L3::SE3 operator*( const L3::SE3& lhs,  const double multiplicand )
    {
        L3::SE3 retval( lhs );

        retval.X( lhs.X()*multiplicand );
        retval.Y( lhs.Y()*multiplicand );
        retval.Z( lhs.Z()*multiplicand );
        retval.R( lhs.R()*multiplicand );
        retval.P( lhs.P()*multiplicand );
        retval.Q( lhs.Q()*multiplicand );

        return retval;
    }


}

template L3::SE3 L3::Estimator::ParticleFilter<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template L3::SE3 L3::Estimator::EKF<double>::operator()(L3::PointCloud<double>*, L3::SE3);

template bool L3::Estimator::ParticleFilter<double>::update(double);
template bool L3::Estimator::EKF<double>::update(double);

template L3::Estimator::EKF<double>::EKF(boost::shared_ptr<L3::Estimator::CostFunction<double> >, boost::shared_ptr<L3::HistogramPyramid<double> >, boost::shared_ptr<L3::VelocityProvider>);

