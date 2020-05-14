#include "Imagery.h"

#define STB_IMAGE_IMPLEMENTATION
#include "imagery/stb_image.h"

namespace common
{
namespace imagery
{
  IMAGE loadImage( const std::string& target )
  {
    if ( !boost::filesystem::exists( target ))
      throw std::runtime_error(target + " does not exist!");
    int width, height, channels;
    unsigned char * data = stbi_load(target.c_str(), &width, &height, &channels, 0);
    return boost::make_shared<common::imagery::Image>( data, width, height, channels );
  }   
}
}
