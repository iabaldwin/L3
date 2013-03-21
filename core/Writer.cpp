#include "Writer.h"

namespace L3
{
namespace IO
{


    Writer::~Writer()
    {
        if (stream.is_open())
            stream.close();
    }

    bool Writer::open( const std::string& f )
    {
        stream.open( f.c_str(), std::ios::out );
        return stream.good();
    }

L3::IO::Writer& operator<<( L3::IO::Writer& o, const std::vector<Pose*>& poses )
{
    if ( !o.stream.good() )
        throw std::exception();

    for( std::vector<L3::Pose*>::const_iterator it=poses.begin(); it!=poses.end(); it++ )
    {
        o.stream << *(*it) << std::endl;
    }

    return o;
}

}
}
