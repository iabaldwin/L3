#ifndef L3_SIMULATOR_H
#define L3_SIMULATOR_H

#include "Datatypes.h"
#include "Iterator.h"

namespace L3
{
namespace Simulator
{
    struct LHLVGenerator : L3::Iterator<L3::LHLV>
    {

        LHLVGenerator( double duration=10.0 ) : L3::Iterator<L3::LHLV>( boost::shared_ptr< L3::SlidingWindow<L3::LHLV > >()),  duration(duration)
        {

        }

        double duration;

        bool update( double t );

    };


}
}

#endif

