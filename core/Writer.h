#ifndef L3_WRITER_H
#define L3_WRITER_H

#include <vector>
#include <iterator>
#include <fstream>

#include <boost/shared_ptr.hpp>

#include "Datatypes.h"


namespace L3
{
    namespace IO
    {

        class Writer 
        {

            public:
                virtual ~Writer();

                bool open( const std::string& f );


            protected:

                std::ofstream stream;

            private:

                friend L3::IO::Writer& operator<<( L3::IO::Writer& o, const std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses );
        
        };
    }
}

#endif
