#ifndef L3_VELOCITY_SOURCE
#define L3_VELOCITY_SOURCE

#include "Core.h"
#include "Filter.h"
#include "ScanMatching.h"

typedef std::deque< std::pair< double, std::vector< double > > > VELOCITY_WINDOW;

struct zipper : std::unary_function< std::pair< double, boost::shared_ptr< L3::LHLV > >, std::pair< double, std::vector< double > > >
{
    zipper()
    {
        data.resize(4);
    }

    std::vector< double > data;

    std::pair< double, std::vector< double > > operator()( std::pair< double, boost::shared_ptr< L3::LHLV > > input )     
    {
        data[0] = input.second->data[9];
        data[3] = input.second->data[3];

        return std::make_pair( input.first, data );
    }

};

namespace L3
{

struct VelocityProvider : Lockable, TemporalObserver
{

    VelocityProvider() : previous_update(0.0)
    {
    }

    VELOCITY_WINDOW raw_velocities;
    VELOCITY_WINDOW filtered_velocities;

    typedef VELOCITY_WINDOW::iterator VELOCITY_ITERATOR;
    
    double previous_update;
};


struct ScanMatchingVelocityProvider : VelocityProvider
{

    ScanMatchingVelocityProvider( L3::ScanMatching::Engine* engine  ) ;

    L3::ScanMatching::Engine* engine;

    Comparator< std::pair< double, Eigen::Matrix4f > > comparator;

    bool update( double time );

};


struct SMVelocityProvider : VelocityProvider
{
    SMVelocityProvider( boost::shared_ptr< L3::ConstantTimeIterator< L3::SMVelocity > > velocity_provider ) : velocity_provider(velocity_provider)
    {
    }
    
    boost::weak_ptr< L3::ConstantTimeIterator< L3::SMVelocity > > velocity_provider;
    
    bool update( double time );

};

struct LHLVVelocityProvider : VelocityProvider
{
    LHLVVelocityProvider( L3::ConstantTimeIterator< L3::LHLV > * iterator ) : iterator(iterator)
    {

    }

    Comparator< std::pair< double, boost::shared_ptr< L3::LHLV > > > comparator;
    Comparator< std::pair< double, std::vector< double > > > velocity_comparator;       
    L3::ConstantTimeIterator< L3::LHLV > * iterator;
    
    zipper z;

    bool update( double time );

};




}

#endif

