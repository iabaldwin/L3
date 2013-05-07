#ifndef L3_SCAN_MATCHING_H
#define L3_SCAN_MATCHING_H

#include "Core.h"
#include "Datatypes.h"
#include "Datatypes.h"
#include "Iterator.h"

#include <boost/shared_array.hpp>

namespace L3
{
namespace ScanMatching
{
    class ScanMatcher  
    {
        public:
          
            ScanMatcher() : initialised(false)
            {
            }

            virtual bool match(  const L3::LMS151& current_scan )
            {
            }

            boost::shared_array< double > scan;
            int scan_points;
            
            boost::shared_array< double > putative;
            int putative_points;

        protected:
            
            bool initialised;
            
                
    };

    class ICP : public ScanMatcher
    {
        public:

            bool match(  const L3::LMS151& current_scan );

    };

    struct Engine : L3::TemporalObserver, Lockable
    {

        Engine( L3::ConstantTimeIterator<L3::LMS151>* windower ) : 
            windower(windower)
        {
            matcher.reset( new ICP() );
        }
            
        std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;

        L3::ConstantTimeIterator<L3::LMS151>* windower;
        
        boost::shared_ptr< ScanMatcher > matcher;

        bool update( double t );

    };

}
}






#endif
