#include "Imagery.h"
namespace L3
{
  namespace Visualisers
  {
    boost::shared_ptr< LocaleRenderer > LocaleRendererFactory::buildLocale( L3::Configuration::Locale& locale )        
    {
      std::stringstream ss;

      char* L3_DIR;
      L3_DIR = getenv ("L3");
      CHECK_NOTNULL(L3_DIR);

      ss << std::string(L3_DIR) << "/media/" << locale.name << ".png";

      ImageData data;
      ImageFactory::Image(ss.str(), data);

      // Image calibration
      image_bounds b;

      // Read from file
      std::string image_parameters = std::string(L3_DIR)  + "/media/" + locale.name + ".dat";
      std::ifstream stream( image_parameters.c_str() );

      if ( !stream.good()) {
        LOG(FATAL) << "No data file!";
        return boost::shared_ptr< LocaleRenderer >();
      }

      double x_offset, y_offset, size, scale, multiplier;
      stream >> x_offset >> y_offset >> size >> scale >> multiplier;
      b.lower_x = 0-x_offset;
      b.lower_y = 0-y_offset;
      b.upper_x = size*scale*multiplier-x_offset;
      b.upper_y = size*scale*multiplier-y_offset;

      return boost::make_shared<LocaleRenderer>( data, b  );
    }
  } // Visualisers
} // L3
