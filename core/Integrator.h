#ifndef L3_INTEGRATOR_H
#define L3_INTEGRATOR_H

#include "Core.h"
#include "Datatypes.h"

namespace L3
{

    struct LengthEstimator 
    {

        LengthEstimator() : initialised(false), previous( L3::SE3::ZERO() )
        {
        }

        bool initialised;
        L3::SE3 previous;

        double operator()( L3::SE3 current )
        {
            if ( !initialised )  
            {
                previous = current;
                initialised = true;
                return 0.0;
            }
            else
            {
                double n = L3::Math::norm( previous, current );
                previous = current;
                return n;
            }

        }
    };
struct LengthEstimatorInterface : L3::LengthEstimator
{
    L3::LengthEstimator estimator;

    double operator()( std::pair< double, boost::shared_ptr<L3::SE3> > element )
    {
        return estimator( *element.second );
    }

};



template <typename InputIterator, typename OutputIterator >
    OutputIterator trajectoryAccumulate( InputIterator begin, InputIterator end, OutputIterator output, double& distance, double required_distance, L3::SE3 start_pose = L3::SE3::ZERO() );

template <typename InputIterator, typename OutputIterator >
    void reverseTrajectoryAccumulate( InputIterator begin, InputIterator end, OutputIterator output, double required_increment, double total_distance, int& written );
}

#include "Integrator.hpp"

#endif

