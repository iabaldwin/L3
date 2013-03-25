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

    
L3::IO::Writer& operator<<( L3::IO::Writer& o, const std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses )
{
    if ( !o.stream.good() )
        throw std::exception();

    for( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::const_iterator it=poses.begin(); it!=poses.end(); it++ )
    {
        o.stream << *(it->second) << std::endl;
    }

    return o;
}

}
}
