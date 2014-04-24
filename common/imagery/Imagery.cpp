#include "Imagery.h"

namespace common
{
namespace imagery
{
    IMAGE loadImage( const std::string& target ) 
    {
        if ( !boost::filesystem::exists( target )) 
            throw std::exception();

        return boost::make_shared<common::imagery::Image>( cvLoadImage( target.c_str() ) );
    }
}
}
