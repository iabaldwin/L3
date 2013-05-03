#include "ScanMatching.h"

namespace L3
{
namespace ScanMatching
{
    bool ICP::match(  const L3::LMS151& scan )
    {
        if ( !initialised )
            this->scan = scan;
    }
    
    bool Engine::update( double t )
    {
        std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;
        this->windower->getWindow( window );

        //TODO:
        //Do we get the closest scan at time, or the next in the sequence?

        //1. Get scan
        //2. Add it to the matcher

        if ( window.size() > 0 )
            matcher->match( *(window.front().second ) ); 
    }


}
}
