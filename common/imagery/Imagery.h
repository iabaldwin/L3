#ifndef COMMON_IMAGERY_H
#define COMMON_IMAGERY_H

#include <flann/flann.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

namespace common
{
namespace imagery
{

  struct Image {
    Image( IplImage* image ) : _image(image), 
    width(image->width), height(image->height), 
    widthStep(image->widthStep ), 
    imageData(image->imageData) {
    }

    IplImage* _image;
    char* imageData;
    int width, height, widthStep;

    ~Image() {
      cvReleaseImage(&_image);
    }

  };

  typedef boost::shared_ptr< Image > IMAGE;

  // Loading
  IMAGE loadImage( const std::string& target ) ;

}
}

#endif
