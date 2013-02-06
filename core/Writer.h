#ifndef L3_IO_H
#define L3_IO_H

#include <vector>
#include <iterator>
#include <fstream>
#include "Datatypes.h"


namespace L3
{
namespace IO
{

class Writer 
{

    public:
        virtual ~Reader()
        {
            if (stream.is_open())
                stream.close();
        }

        bool open( const std::string& f )
        {
            stream.open( f.c_str(), std::ios::in );
            return stream.good();
        }

        size_t write()
        {
            std::copy( std::istream_iterator<double>( stream ),
                        std::istream_iterator<double>(),
                        std::back_inserter( raw ) );

            return raw.size();
        }


};


}
}

#endif
