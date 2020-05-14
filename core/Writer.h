#pragma once

#include <vector>
#include <iterator>
#include <fstream>

#include "Core.h"
#include "Datatypes.h"

namespace L3
{
  namespace IO
  {
    class Writer 
    {

      public:
        virtual bool open( const std::string& f ) = 0;

        virtual ~Writer()
        {
          if (stream.is_open())
            stream.close();
        }

      protected:
        std::ofstream stream;
    };


    template <typename T>
      class BinaryWriter : Writer
    {
      public:

        virtual bool open( const std::string& f )
        {
          this->stream.open( f.c_str(), std::ios::out | std::ios::binary );

          return this->stream.good();
        }

        virtual size_t write( std::vector< std::pair< double, boost::shared_ptr<T> > >& data ) ;
    };
  } // IO
} // L3
