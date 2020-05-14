#pragma once

#include <boost/random.hpp>

#include "Iterator.h"
#include "Estimator.h"
#include "VelocityProvider.h"
#include "BayesFilter/unsFlt.hpp"

namespace L3
{

  inline L3::SE3 operator+(const L3::SE3& lhs,  const L3::SE3& rhs) {
    L3::SE3 retval;
    retval.X(lhs.X() + rhs.X());
    retval.Y(lhs.Y() + rhs.Y());
    retval.Z(lhs.Z() + rhs.Z());
    retval.R(lhs.R() + rhs.R());
    retval.P(lhs.P() + rhs.P());
    retval.Q(lhs.Q() + rhs.Q());
    return retval;
  }

  inline L3::SE3 operator/(const L3::SE3& lhs,  const double divisor) {
    L3::SE3 retval(lhs);
    retval.X(lhs.X()/divisor);
    retval.Y(lhs.Y()/divisor);
    retval.Z(lhs.Z()/divisor);
    retval.R(lhs.R()/divisor);
    retval.P(lhs.P()/divisor);
    retval.Q(lhs.Q()/divisor);
    return retval;
  }

  inline L3::SE3 operator*(const L3::SE3& lhs,  const double multiplicand) {
    L3::SE3 retval(lhs);
    retval.X(lhs.X()*multiplicand);
    retval.Y(lhs.Y()*multiplicand);
    retval.Z(lhs.Z()*multiplicand);
    retval.R(lhs.R()*multiplicand);
    retval.P(lhs.P()*multiplicand);
    retval.Q(lhs.Q()*multiplicand);
    return retval;
  }

  inline double norm(const L3::SE3& a, const L3::SE3& b) {
    return sqrt(pow(a.X() - b.X(),2)  + pow(a.Y() - b.Y(),2));
  }

  namespace Estimator
  {
    template <typename T>
      struct Filter
      {
        Filter(boost::shared_ptr< L3::VelocityProvider > iterator)
          : iterator(iterator)
        {
        }

        boost::weak_ptr < L3::VelocityProvider > iterator ;
      };


    struct PredictionModel : Bayesian_filter::Additive_predict_model, TemporalObserver
    {
      PredictionModel(boost::shared_ptr< L3::VelocityProvider > iterator)
        :  Bayesian_filter::Additive_predict_model(3,3),
        iterator(iterator),
        last_update(0.0),
        current_update(0.0) {
          _x.reset(new Bayesian_filter_matrix::Vec(3));

          Bayesian_filter_matrix::identity (G);
          q[0] = 1;
          q[1] = 1;
          q[2] = .1;
          G(0,0) = .1;
          G(1,1) = .1;
          G(2,2) = .01;
        }

      mutable double last_update, current_update;

      boost::weak_ptr< L3::VelocityProvider > iterator ;

      boost::shared_ptr< Bayesian_filter_matrix::Vec > _x;

      mutable L3::SE3 prediction, delta;

      mutable SE3 check_pose;

      bool update(double time) {
        if (last_update == 0) {
          last_update = time;
          return false;
        }

        boost::shared_ptr< L3::VelocityProvider > velocity_ptr = this->iterator.lock();

        if (!velocity_ptr) {
          return false;
        }

        L3::ReadLock master(velocity_ptr->mutex);

        L3::VelocityProvider::VELOCITY_COMPARATOR c;

        L3::VelocityProvider::VELOCITY_ITERATOR index = std::lower_bound(velocity_ptr->filtered_velocities.begin(),
            velocity_ptr->filtered_velocities.end(),
            last_update,
            c);

        if (index->first == last_update) {
          index++;
        }

        if(index==velocity_ptr->filtered_velocities.begin()) {
          return false;
        }

        double x=0, y=0, z=0;
        double r=0, p=0, q=0;

        double w1=0, w2=0, w3=0;
        double lin_vel=0, x_vel=0, y_vel=0, z_vel=0;

        std::deque < std::pair< double, boost::shared_ptr< L3::SE3 > > > _window;

        // Origin start
        _window.push_back(std::make_pair(index->first, boost::make_shared< L3::SE3 > ()));

        std::deque < std::pair< double, boost::shared_ptr< L3::SE3 > > >::iterator _window_iterator = _window.begin();

        boost::shared_ptr< L3::SE3 > previous_pose = _window_iterator->second;
        boost::shared_ptr< L3::SE3 > current_pose;

        static int counter = 0;

        // Integrate
        while(index != velocity_ptr->filtered_velocities.end()) {
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
          current_pose =  boost::make_shared<L3::SE3>(x, y, z, r, p, q);

          previous_pose = current_pose;

          index++;
        }

        if(current_pose) {
          delta = *current_pose;
        }

        last_update = velocity_ptr->filtered_velocities.back().first;

        master.unlock();

        return true;
      }

      //const Bayesian_filter_matrix::Vec& f (const Bayesian_filter_matrix::Vec &x) const;

      const Bayesian_filter_matrix::Vec& f (const Bayesian_filter_matrix::Vec &x) const {
        if(delta == L3::SE3::ZERO()) {
          return x;
        }

        L3::SE3 pose(x[0], x[1], 0, 0, 0, x[2]);

        Eigen::Matrix4f tmp1(pose.getHomogeneous());
        Eigen::Matrix4f tmp2(delta.getHomogeneous());
        tmp1 *= tmp2;

        // Is it here?
        prediction = L3::Utils::Math::poseFromRotation(tmp1);

        // Wrap
        if (pose.Q() - prediction.Q() > M_PI) {
          double q = prediction.Q();
          q += 2*M_PI;
          prediction.Q(q);
        }

        if (pose.Q() - prediction.Q() < -1*M_PI) {
          double q = prediction.Q();
          q -= 2*M_PI;
          prediction.Q(q);
        }

        Bayesian_filter_matrix::Vec estimate(3);

        estimate[0] = prediction.X();
        estimate[1] = prediction.Y();
        estimate[2] = prediction.Q();

        _x->assign(estimate);

        // DBG
        static int counter = 0;
        if ((++counter%7)== 0) {
          Eigen::Matrix4f& check_pose_m = check_pose.getHomogeneous();
          check_pose_m *= tmp2;
        }
        // DBG

        return *_x;
      }
    };

    struct ObservationModel : Bayesian_filter::Linear_uncorrelated_observe_model
    {
      ObservationModel () : Bayesian_filter::Linear_uncorrelated_observe_model(3,3) {
        Bayesian_filter_matrix::identity( Hx);
        Zv[0] = 4*1;
        Zv[1] = 4*1;
        Zv[2] = 4*0.08;
      }
    };

    template <typename T>
      struct UKF : Filter<T>, Algorithm<T>, L3::TemporalObserver
    {
      UKF(boost::shared_ptr<CostFunction<T> > cost_function,
          boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid,
          boost::shared_ptr< L3::VelocityProvider > iterator);

      boost::shared_ptr< Bayesian_filter::Unscented_scheme > ukf;

      boost::shared_ptr< Bayesian_filter_matrix::Vec >       x_init;
      boost::shared_ptr< Bayesian_filter_matrix::SymMatrix>  X_init;

      boost::shared_ptr< PredictionModel >    prediction_model;
      boost::shared_ptr< ObservationModel >   observation_model;

      L3::Timing::ChronoTimer timer;

      bool initialised;
      double previous_time, current_time;

      std::vector< double > sigma_points;

      boost::shared_ptr< Minimisation<T> > minimiser;

      SE3 operator()(PointCloud<T>* swathe, SE3 estimate);

      bool update(double time);

      std::string name()
      {
        return "UKF";
      }
    };

    template <typename T>
      struct ParticleFilter : Filter<T>, Algorithm<T>, L3::TemporalObserver
    {
      ParticleFilter(boost::shared_ptr<CostFunction<T> > cost_function,
          boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid,
          boost::shared_ptr< L3::VelocityProvider > iterator,
          int num_particles = 800)
        : Filter<T>(iterator),
        Algorithm<T>(cost_function),
        previous_time(0.0),
        pyramid(experience_pyramid),
        initialised(false),
        num_particles(num_particles)
      {
        sampled_swathe = boost::make_shared< PointCloud<T> >();

        L3::allocate(sampled_swathe.get(), 4*1000);

        linear_uncertainty = .75;
        rotational_uncertainty = .15;

      }

      bool initialised;
      int num_particles;
      double previous_time, current_time;
      double linear_uncertainty, rotational_uncertainty;

      boost::mt19937 rng;
      tbb::task_group group;

      boost::shared_ptr< PointCloud<T> > sampled_swathe;
      boost::shared_ptr< HistogramPyramid<T> > pyramid;

      std::vector< double > weights;
      std::vector< L3::SE3 > hypotheses;

      typedef std::vector< L3::SE3 >::iterator PARTICLE_ITERATOR;

      // Search structure
      Comparator< VELOCITY_WINDOW::value_type > comparator;

      SE3 operator()(PointCloud<T>* swathe, SE3 estimate);

      bool update(double time);

      std::string name()
      {
        return "ParticleFilter";
      }
    };

  } // Estimator
} // L3

#include "Filter.hpp"
