#pragma once

#include "boost/shared_ptr.hpp"
#include "boost/filesystem.hpp"
#include "boost/make_shared.hpp"

namespace common
{
  namespace imagery
  {
    struct Image {
      Image( unsigned char* data , int width, int height, int channels ) : imageData(data),
             width{width}, height{height},
             widthStep{channels * width} {
      }

      unsigned char* imageData{nullptr};
      int width{-1}, height{-1}, widthStep{-1};
    };

    typedef boost::shared_ptr< Image > IMAGE;
  
    IMAGE loadImage( const std::string& target );

  } // imagery
} // common
