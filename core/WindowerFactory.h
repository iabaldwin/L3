#pragma once

#include <vector>

#include "Windower.h"

typedef unsigned char BYTE;

namespace L3
{
  template <typename T>
    class WindowerFactory
    {
      public:
        static boost::shared_ptr<SlidingWindow<T> > constantTimeWindow( const std::string& file, float time )
        {
          std::ifstream file_input( file.c_str() );

          // Read N bytes, try to determine if it is binary or not
          std::vector<BYTE> sample( 1024 );

          if( not file_input.good() )
          {
            LOG(ERROR) << "No such file: [" << file.c_str() << "]";
            return boost::shared_ptr< SlidingWindow<T> >();
          }

          file_input.read( (char*)(&sample[0]), 1024 );
          file_input.close();

          std::vector<BYTE>::iterator val_max = std::max_element( sample.begin(), sample.end() );
          std::vector<BYTE>::iterator val_min = std::min_element( sample.begin(), sample.end() );

          int int_max = int(*val_max);
          int int_min = int(*val_min);

          return ( int_max > 127  || int_min< 10 ) ?
            boost::make_shared< L3::SlidingWindowBinary<T> >( file, time ) :
            boost::make_shared< L3::SlidingWindow<T> >( file, time ) ;
        }
    };
}
