#ifndef L3_SCAN_MATCHING_H
#define L3_SCAN_MATCHING_H

#include "Core.h"
#include "Datatypes.h"
#include "Datatypes.h"
#include "Iterator.h"

namespace L3
{
namespace ScanMatching
{
    class ScanMatcher 
    {
        public:
            
            virtual bool match(  const L3::LMS151& scan )
            {

            }

        protected:
            L3::LMS151 scan;
            bool initialised;
    };

    class ICP : public ScanMatcher
    {
        public:

            bool match( const L3::LMS151& scan );

    };

    struct Engine : public L3::TemporalObserver
    {

        Engine( L3::ConstantTimeIterator<L3::LMS151>* windower ) : 
            windower(windower)
        {
            matcher.reset( new ICP() );
        }

        L3::ConstantTimeIterator<L3::LMS151>* windower;
        
        boost::shared_ptr< ScanMatcher > matcher;

        bool update( double t );

    };

}
}






#endif
