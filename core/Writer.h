#ifndef L3_IO_WRITER_H
#define L3_IO_WRITER_H

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
        virtual ~Writer()
        {
            if (stream.is_open())
                stream.close();
        }

        bool open( const std::string& f )
        {
            stream.open( f.c_str(), std::ios::out );
            return stream.good();
        }

    protected:

        std::ofstream stream;


    private:
        friend  L3::IO::Writer& operator<<( L3::IO::Writer& o, const std::vector<Pose*>& poses );

};

L3::IO::Writer& operator<<( L3::IO::Writer& o, const std::vector<Pose*>& poses )
{
    if ( !o.stream.good() )
        throw std::exception();

    for( std::vector<L3::Pose*>::const_iterator it=poses.begin(); it!=poses.end(); it++ )
    {
        //std::cout << o.stream << std::endl;
        //std::cout << *(*it);
        o.stream << *(*it) << std::endl;
    }

    return o;
}

}
}

#endif
