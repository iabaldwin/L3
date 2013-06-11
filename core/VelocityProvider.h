#ifndef L3_VELOCITY_SOURCE
#define L3_VELOCITY_SOURCE

#include "Core.h"
#include "ScanMatching.h"

    
//typedef std::deque< std::pair< double, std::pair< double, double > > > VELOCITY_WINDOW;
//typedef std::vector< std::pair< double, std::pair< double, double > > > VELOCITY_WINDOW;
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

    ScanMatchingVelocityProvider( L3::ScanMatching::Engine* engine  ) : engine(engine)
    {

    }

    L3::ScanMatching::Engine* engine ;

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

