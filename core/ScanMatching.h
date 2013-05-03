#ifndef L3_SCAN_MATCHING_H
#define L3_SCAN_MATCHING_H

namespace L3
{
namespace ScanMatching
{
    
    class ScanMatcher : public L3::Observer
    {
        public:
            virtual bool initialise(  const L3::LMS151& scan )
            {


            }

            virtual bool match(  const L3::LMS151& scan )
            {

            }

    };

    class ICP : public ScanMatcher
    {
        public:

            virtual bool match(  const L3::LMS151& scan )
            {

            }

            bool update( double time )
            {


            }

    };

}
}






#endif
