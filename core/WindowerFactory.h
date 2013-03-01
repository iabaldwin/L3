#ifndef L3_WINDOWER_FACTORY
#define L3_WINDOWER_FACTORY

#include <vector>

#include "Windower.h"

typedef char BYTE;

namespace L3
{

    template <typename T>
    class WindowerFactory
    {

        public:
            static boost::shared_ptr<SlidingWindow<T> > constantTimeWindow( const std::string& file, float time )
            {
                std::vector<BYTE> sample( 1024 );

                std::ifstream file_input( file.c_str() );

                file_input.read( (char*)(&sample[0]), 1024 );

                file_input.close();

                std::vector<BYTE>::iterator val_max = std::max_element( sample.begin(), sample.end() );
                std::vector<BYTE>::iterator val_min = std::min_element( sample.begin(), sample.end() );

                int int_max = int(*val_max);
                int int_min = int(*val_min);


                return ( int_max > 127  || int_min< 10 ) ? 
                    boost::shared_ptr< L3::SlidingWindow<T> >( new L3::SlidingWindowBinary<T>( file, time ) ) :
                    boost::shared_ptr< L3::SlidingWindow<T> >( new L3::SlidingWindow<T>( file, time ) ) ;
            }

    };

}

#endif

