#include "Filter.h"

#include "Integrator.h"
#include "Utils.h"

namespace L3
{

    namespace Estimator
    {

        Bayesian_filter_matrix::Vec adapt( const L3::SE3& pose )
        {
            Bayesian_filter_matrix::Vec ret_val(3);

            ret_val[0] = pose.X();
            ret_val[1] = pose.Y();
            ret_val[2] = pose.Q();

            return ret_val;

        }
        
        L3::SE3 adapt( const Bayesian_filter_matrix::Vec&  pose )
        {
            L3::SE3 ret_val;
            ret_val.X( pose[0] );
            ret_val.Y( pose[1] );
            ret_val.Q( pose[2] );
       
            return ret_val;
        }

        PredictionModel::PredictionModel( boost::shared_ptr< L3::VelocityProvider > iterator ) 
            :  Bayesian_filter::Additive_predict_model(3,3),
            iterator(iterator),
            last_update(0.0),
            current_update(0.0)
        {
            _x.reset( new Bayesian_filter_matrix::Vec(3) );

            q[0] = 2;
            q[1] = 2;
            q[2] = .2;

            G(0,0) = .1;
            G(1,1) = .1;
            G(2,2) = .01;
        }

        const Bayesian_filter_matrix::Vec& PredictionModel::f (const Bayesian_filter_matrix::Vec &x) const
        {
            if( delta == L3::SE3::ZERO() )  // No update
                return x;

            L3::SE3 pose( x[0], x[1], 0, 0, 0, x[2] );

            Eigen::Matrix4f tmp1( pose.getHomogeneous() );
            Eigen::Matrix4f tmp2( delta.getHomogeneous() );
            tmp1 *= tmp2;

            prediction = L3::Utils::Math::poseFromRotation( tmp1 );

            Bayesian_filter_matrix::Vec estimate(3);

            estimate[0] = prediction.X();
            estimate[1] = prediction.Y();
            estimate[2] = prediction.Q();

            _x->assign( estimate );

            return *_x;


        }

        bool PredictionModel::update( double time )
        {
            if ( last_update == 0 )
            {
                last_update = time;  
              
                return false;
            }

            boost::shared_ptr< L3::VelocityProvider > iterator_ptr = this->iterator.lock();


            L3::VelocityProvider::VELOCITY_COMPARATOR c;

            L3::VelocityProvider::VELOCITY_ITERATOR index = std::lower_bound( iterator_ptr->filtered_velocities.begin(),
                    iterator_ptr->filtered_velocities.end(),
                    last_update,
                    c );

            if( index->first == last_update )
                index++;

            if ( index == iterator_ptr->filtered_velocities.begin() )
                return false;

            L3::SE3 _delta;

            double x=0, y=0, z=0;
            double r=0, p=0, q=0;

            double w1=0, w2=0, w3=0;
            double lin_vel=0, x_vel=0, y_vel=0, z_vel=0;

            std::deque < std::pair< double, boost::shared_ptr< L3::SE3 > > > _window;

            // Origin start
            _window.push_back( std::make_pair( index->first, boost::make_shared< L3::SE3 > () ) );

            std::deque < std::pair< double, boost::shared_ptr< L3::SE3 > > >::iterator _window_iterator = _window.begin();

            boost::shared_ptr< L3::SE3 > previous_pose = _window_iterator->second, current_pose;


            // Integrate
            while( index != iterator_ptr->filtered_velocities.end() )
            {
                double dt = index->first - (index-1)->first;

                w1 = index->second[1];
                w2 = index->second[2];
                w3 = index->second[3];

                r = previous_pose->R() + w1*dt;
                p = previous_pose->P() + w2*dt;
                q = previous_pose->Q() + (-1*w3)*dt;

                lin_vel = index->second[0];

                x_vel = lin_vel * sin(q);  
                y_vel = lin_vel * cos(q);  
                z_vel = lin_vel * sin(p);  


                x = previous_pose->X() + -1*x_vel*dt;
                y = previous_pose->Y() + y_vel*dt;
                z = previous_pose->Z() + z_vel*dt;

                // Log
                current_pose =  boost::make_shared<L3::SE3>( x, y, z, r, p, q );

                previous_pose = current_pose;

                index++;
            }

            if( current_pose ) 
                delta = *current_pose;
            else
            {
                // No update
                delta = L3::SE3::ZERO();
                return false;
            }

            last_update = iterator_ptr->filtered_velocities.back().first;
        }

        ObservationModel::ObservationModel () : Bayesian_filter::Linear_uncorrelated_observe_model(3,3)
        {
            Hx(0,0) = 1.;
            Hx(1,1) = 1.;
            Hx(2,2) = 1.;
            
            Zv[0] = .01;
            Zv[1] = .01;
            Zv[2] = .001;
        }


        template <typename T>
            UKF<T>::UKF( boost::shared_ptr<CostFunction<T> > cost_function,  
                    boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, 
                    boost::shared_ptr< L3::VelocityProvider > iterator )
            : Filter<T>(iterator), 
            Algorithm<T>(cost_function, 2 ),
            initialised(false)
        {
            ukf.reset( new Bayesian_filter::Unscented_scheme(3) );

            x_init.reset( new Bayesian_filter_matrix::Vec(3) );
            X_init.reset( new Bayesian_filter_matrix::SymMatrix(3,3) );

            prediction_model  = boost::make_shared< PredictionModel >(  iterator );
            observation_model = boost::make_shared< ObservationModel >();

            minimiser = boost::make_shared< Minimisation<T> >( cost_function, experience_pyramid );

            sigma_points.resize( (2*3 + 1)*3 );
                
            timer.begin();
        }

        template <typename T>
            SE3 UKF<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
            {
                if( !initialised )
                {
                    (*x_init)[0]= estimate.X();
                    (*x_init)[1]= estimate.Y();
                    (*x_init)[2]= estimate.Q();

                    ukf->init_kalman (*x_init, *X_init);

                    boost::shared_ptr< L3::VelocityProvider > iterator_ptr = this->iterator.lock() ;
                    prediction_model->last_update = iterator_ptr->filtered_velocities.back().first;

                    initialised = true;

                    return estimate;
                }

                // Always predict 
                ukf->predict( *prediction_model);
                ukf->update();

                // Sometimes update
                if( timer.elapsed() > 1.0/this->fundamental_frequency )
                {
                    // Produce measurement
                    L3::SE3 z = minimiser->operator()( swathe, estimate );

                    Bayesian_filter_matrix::Vec z_vec = adapt(z);

                    ukf->observe( *observation_model, z_vec );
                        
                    ukf->update();
                }

                double* ptr = &sigma_points[0];
                for( int j=0; j< ukf->XX.size2(); j++ )
                    for( int i=0; i< ukf->XX.size1(); i++ )
                        *ptr++ = ukf->XX(i,j);

                current_estimate = adapt( ukf->x );

                return current_estimate;
            }

        template <typename T>
            bool UKF<T>::update( double time )
            {
                prediction_model->update( time );

                return true;
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

                /*
                 *Uncertainty generators
                 */
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

                double q, x_vel, y_vel, velocity_delta, x, y;

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
template L3::SE3 L3::Estimator::UKF<double>::operator()(L3::PointCloud<double>*, L3::SE3);

template bool L3::Estimator::ParticleFilter<double>::update(double);
template bool L3::Estimator::UKF<double>::update(double);

template L3::Estimator::UKF<double>::UKF(boost::shared_ptr<L3::Estimator::CostFunction<double> >, boost::shared_ptr<L3::HistogramPyramid<double> >, boost::shared_ptr<L3::VelocityProvider>);

