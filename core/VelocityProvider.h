#ifndef L3_VELOCITY_SOURCE
#define L3_VELOCITY_SOURCE

#include "Core.h"
#include "ScanMatching.h"

namespace L3
{

struct VelocityProvider, Lockable, TemporalObserver
{
    std::deque< double, std::pair< double, double > > velocities;
};


struct ScanMatchingVelocityProvider : VelocityProvider
{

    ScanMatchingVelocityProvider( L3::ScanMatching::Engine* engine  ) : engine(engine)
    {

    }

    L3::ScanMatching::Engine* engine ;

    void update( double time )
    {

        velocities.clear();

        L3::ReadLock lock( engine->mutex );
        
        lock.unlock();





    }

};

}

#endif

