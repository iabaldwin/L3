#include "Imagery.h"
namespace L3
{
namespace Visualisers
{
  boost::shared_ptr< LocaleRenderer > LocaleRendererFactory::buildLocale( L3::Configuration::Locale& locale )        
  {
    std::stringstream ss;

    char* L3_DIR;
    L3_DIR = getenv ("L3_DIR");
    assert (L3_DIR!=NULL);

    ss << std::string(L3_DIR) << "/media/" << locale.name << ".png";

    ImageData data;
    ImageFactory::Image(ss.str(), data);

    // Image calibration
    image_bounds b;

    // Read from file
    std::string image_parameters = std::string(L3_DIR)  + "/media/" + locale.name + ".dat";
    std::ifstream stream( image_parameters.c_str() );

    if ( !stream.good())
      return boost::shared_ptr< LocaleRenderer >();

    double x_offset, y_offset, size, scale, multiplier;
    stream >> x_offset >> y_offset >> size >> scale >> multiplier;
    b.lower_x = 0-x_offset;
    b.lower_y = 0-y_offset;
    b.upper_x = size*scale*multiplier-x_offset;
    b.upper_y = size*scale*multiplier-y_offset;

    return boost::make_shared<LocaleRenderer>( data, b  );
  }

  boost::shared_ptr< LocaleRenderer > LocaleRendererFactory::buildMap( L3::Configuration::Locale& locale )        
  {
    // This should be embedded in the configuration     
    std::string image_target = "/Users/ian/Documents/map_red.png";

    ImageData data;
    ImageFactory::Image(image_target, data);

    // Image calibration
    image_bounds b;

    double x_offset = 185;
    double y_offset = 125;

    b.lower_x = 0-x_offset;
    b.lower_y = 0-y_offset;
    b.upper_x = 100.0-x_offset;
    b.upper_y = 100.0-y_offset;

    return boost::make_shared<LocaleRenderer>( data, b  );
  }

}
}
