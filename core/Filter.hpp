#pragma once

#include "Integrator.h"
#include "Utils.h"

#include <boost/numeric/ublas/io.hpp>

namespace L3
{
  namespace Estimator
  {
    inline Bayesian_filter_matrix::Vec adapt(const L3::SE3& pose) {
      Bayesian_filter_matrix::Vec ret_val(3);
      ret_val[0] = pose.X();
      ret_val[1] = pose.Y();
      ret_val[2] = pose.Q();
      return ret_val;
    }

    inline L3::SE3 adapt(const Bayesian_filter_matrix::Vec&  pose) {
      L3::SE3 ret_val;
      ret_val.X(pose[0]);
      ret_val.Y(pose[1]);
      ret_val.Q(pose[2]);
      return ret_val;
    }

    template <typename T>
      UKF<T>::UKF(boost::shared_ptr<CostFunction<T> > cost_function,  
          boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, 
          boost::shared_ptr< L3::VelocityProvider > iterator)
      : Filter<T>(iterator), 
      Algorithm<T>(cost_function, 25),
      initialised(false) {
      ukf.reset(new Bayesian_filter::Unscented_scheme(3));

      x_init.reset(new Bayesian_filter_matrix::Vec(3));
      X_init.reset(new Bayesian_filter_matrix::SymMatrix(3,3));

      prediction_model  = boost::make_shared< PredictionModel >( iterator);
      observation_model = boost::make_shared< ObservationModel >();

      minimiser = boost::make_shared< Minimisation<T> >(cost_function, experience_pyramid, 1 , 40);

      sigma_points.resize((2*3 + 1)*3);

      timer.begin();

      this->fundamental_frequency = 20.0;
    }

    template <typename T>
      SE3 UKF<T>::operator()(PointCloud<T>* swathe, SE3 estimate) {
        if(!initialised) {
          (*x_init)[0]= estimate.X();
          (*x_init)[1]= estimate.Y();
          (*x_init)[2]= estimate.Q();

          Bayesian_filter_matrix::identity( *X_init);

          (*X_init)(0,0) = .1;
          (*X_init)(1,1) = .1;
          (*X_init)(2,2) = .1;

          ukf->init_kalman (*x_init, *X_init);

          boost::shared_ptr< L3::VelocityProvider > velocity_ptr = this->iterator.lock() ;

          if(!velocity_ptr->filtered_velocities.empty()) {
            prediction_model->last_update = velocity_ptr->filtered_velocities.back().first;
          } else {
            return estimate;
          }

          initialised = true;

          //DBG
          //prediction_model->check_pose = estimate;
          //DBG

          return estimate;
        }

        ukf->predict(*prediction_model);
        ukf->update();

        double* ptr = &sigma_points[0];
        for(int j=0; j< ukf->XX.size2(); j++)
          for(int i=0; i< ukf->XX.size1(); i++)
            *ptr++ = ukf->XX(i,j);

        if(timer.elapsed() > 1.0/this->fundamental_frequency) {
          timer.begin();

          // Produce measurement
          L3::SE3 z = minimiser->operator()(swathe, estimate);

          Bayesian_filter_matrix::Vec z_vec = adapt(z);

          L3::SE3 predicted = adapt(ukf->x);

          double innov = norm(predicted, z);

          if (innov < 10) {
            // Observe
            ukf->observe(*observation_model, z_vec);

            // Produce estimate and uncertainty
            ukf->update();

            L3::SE3 estimated = adapt(ukf->x);

            if(fabs(predicted.Q() - estimated.Q()) > M_PI/2.0) {
              // Tracking failure
              LOG(ERROR)<< "Tracking failure";
              exit(EXIT_FAILURE);
            }
          }
        }

        *(this->current_prediction) = adapt(ukf->x);

        return *(this->current_prediction);
      }

    template <typename T>
      bool UKF<T>::update(double time) {
        prediction_model->update(time);
        return true;
      }

    template <typename T>
      SE3 ParticleFilter<T>::operator()(PointCloud<T>* swathe, SE3 estimate) {
        boost::shared_ptr< L3::VelocityProvider > velocity_provider = this->iterator.lock();

        L3::ReadLock master (velocity_provider->mutex);

        int _num_particles = this->num_particles;

        double _linear_uncertainty = this->linear_uncertainty;
        double _rotational_uncertainty = this->rotational_uncertainty;

        hypotheses.resize(_num_particles);

        if (!initialised && !(estimate == L3::SE3::ZERO())) {
          // Generate the initial spread of particles
          boost::normal_distribution<> normal_x(0.0, 1.0);
          boost::normal_distribution<> normal_y(0.0, 1.0);
          boost::normal_distribution<> normal_theta(0.0, 0.1);

          boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > x_generator(rng, normal_x);
          boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > y_generator(rng, normal_y);
          boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > theta_generator(rng, normal_theta );

          for(PARTICLE_ITERATOR it = hypotheses.begin();
              it != hypotheses.end();
              it++) {
            *it = L3::SE3(x_generator()+estimate.X(), y_generator()+estimate.Y(), 0, 0, 0, theta_generator()+estimate.Q());
          }
          initialised = true;
          return estimate;
        }

        // Find the new data, between the last update time and now
        VELOCITY_WINDOW_ITERATOR index = std::lower_bound(velocity_provider->filtered_velocities.begin(), 
            velocity_provider->filtered_velocities.end(), 
            previous_time,
            comparator);

        VELOCITY_WINDOW _window_delta_buffer;

        // Add in the newly-seen data
        _window_delta_buffer.assign(index, velocity_provider->filtered_velocities.end());

        if (_window_delta_buffer.empty()) {
          return estimate;
        }

        double mean_linear_velocity = 0.0;
        double mean_rotational_velocity = 0.0;

        // Calculate average rotational and linear velocity
        for(VELOCITY_WINDOW_ITERATOR it = _window_delta_buffer.begin();
            it!= _window_delta_buffer.end();
            it++) {
          mean_linear_velocity += it->second[0];
          mean_rotational_velocity += it->second[3];
        }

        // Compute the average velocities
        mean_linear_velocity /= _window_delta_buffer.size();
        mean_rotational_velocity /= _window_delta_buffer.size();

        std::vector<double> results(_num_particles); 
        std::vector<double>::iterator result_iterator = results.begin();

        int pyramid_index = 0;

        L3::ReadLock histogram_lock((*this->pyramid)[pyramid_index]->mutex);

        /*
         *Uncertainty generators
         */
        boost::normal_distribution<> linear_velocity_plant_uncertainty(0.0, _linear_uncertainty);
        boost::normal_distribution<> rotational_velocity_plant_uncertainty(0.0, _rotational_uncertainty);

        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > linear_velocity_uncertainty_generator(rng, linear_velocity_plant_uncertainty);
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > rotational_velocity_uncertainty_generator(rng, rotational_velocity_plant_uncertainty);

        // Compute time elapsed
        double dt = current_time - previous_time;

        std::vector< L3::SE3 > delta;
        delta.reserve(hypotheses.size());

        previous_time = current_time;

        if (dt > 1) {
          return estimate;
        }

        if(!L3::sample(swathe, this->sampled_swathe.get(), 1*1000, false)) {
          throw std::exception();
        }

        double q, x_vel, y_vel, velocity_delta, x, y;

        // Apply the constant average velocities to the particles
        for(PARTICLE_ITERATOR it = hypotheses.begin();
            it != hypotheses.end();
            it++) {
          q = it->Q() + ((-1*mean_rotational_velocity) + rotational_velocity_uncertainty_generator())*dt;

          velocity_delta = mean_linear_velocity + linear_velocity_uncertainty_generator() ;

          x_vel = (velocity_delta * sin(q));  
          y_vel = (velocity_delta * cos(q));

          x = it->X() + -1*x_vel*dt;
          y = it->Y() + y_vel*dt;

          delta.push_back(L3::SE3(x, y, 0, 0, 0, q));

          group.run(Hypothesis(this->sampled_swathe.get(), &*it, (*this->pyramid)[pyramid_index].get(), this->cost_function.get(), result_iterator++));
        }

        group.wait();

        histogram_lock.unlock();

        hypotheses.swap(delta);

        std::vector< double > _weights;
        _weights.assign(results.begin(), results.end());

        double sum = std::accumulate(results.begin(), results.end(), 0.0);

        // Normalize
        weights.resize(_weights.size());
        std::transform(_weights.begin(), _weights.end(), weights.begin(), std::bind2nd(std::divides<double>(), sum));

        (*this->current_prediction) = std::inner_product(hypotheses.begin(), hypotheses.end(), weights.begin(), L3::SE3::ZERO());

        // Build CDF 
        std::vector< double > cdf(weights.size());
        std::partial_sum(weights.begin(), weights.end(), cdf.begin());

        boost::uniform_01<> uniform;
        std::vector< L3::SE3 > resampled;
        resampled.reserve(hypotheses.size());

        //Sample
        for(int i=0; i<_num_particles; i++) {
          double d = uniform(rng);
          resampled.push_back(hypotheses[ std::distance(cdf.begin(), std::lower_bound(cdf.begin(), cdf.end(), d))]); 
        }

        hypotheses.swap(resampled);

        master.unlock();

        return *(this->current_prediction);
      }

    template <typename T>
      bool ParticleFilter<T>::update(double time) {
        current_time = time;
        return true;
      }
  }

  }
