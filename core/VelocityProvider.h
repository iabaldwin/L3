#ifndef L3_VELOCITY_SOURCE
#define L3_VELOCITY_SOURCE

#include "Core.h"
#include "Filter.h"
#include "ScanMatching.h"

typedef std::deque< std::pair< double, std::vector< double > > > VELOCITY_WINDOW;

namespace L3
{

struct VelocityProvider : Lockable, TemporalObserver
{

    VELOCITY_WINDOW raw_velocities;
    VELOCITY_WINDOW filtered_velocities;

    typedef VELOCITY_WINDOW::iterator VELOCITY_ITERATOR;
};


struct ScanMatchingVelocityProvider : VelocityProvider
{

    ScanMatchingVelocityProvider( L3::ScanMatching::Engine* engine  ) ;

    L3::ScanMatching::Engine* engine;

    boost::shared_ptr< L3::Estimator::AlphaBetaFilter > _linear_velocity_filter;
    boost::shared_ptr< L3::Estimator::AlphaBetaFilter > _rotational_velocity_filter;

    bool update( double time );

};

struct LHLVVelocityProvider : VelocityProvider
{
    LHLVVelocityProvider( L3::ConstantTimeIterator< L3::LHLV > * iterator ) : iterator(iterator)
    {

    }

    L3::ConstantTimeIterator< L3::LHLV > * iterator;

    bool update( double time );

};




}

#endif

