#pragma once

#include <deque>
#include <iostream>

#include <glv.h>
#include <GL/glut.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace L3
{
  namespace Visualisers
  {
    template <typename T>
      struct variable_lock
      {
        explicit variable_lock( T& t ) : t(t)
        {
        }

        T& t;
      };

    const GLubyte mask[] =   {0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55};


    struct ColorCycler
    {
      ColorCycler() : counter(0)
      {
        colors.push_back( glv::Color( 1, 0, 0 ) );
        colors.push_back( glv::Color( 0, 1, 0 ) );
        colors.push_back( glv::Color( 0, 0, 1 ) );
      }

      int counter;

      std::deque< glv::Color > colors;

      glv::Color operator()()
      {
        return colors[counter++%3];
      }

    };

    struct ColorMap
    {
      virtual std::pair< glv::Color, glv::Color> getBounds( float v ) = 0;
    };

    struct Jet : ColorMap
    {

      Jet()
      {
        interpolant_values[0] = 0.0;
        interpolant_values[1] = 0.5;
        interpolant_values[2] = 1.0;

        colors[0] = glv::Color( 0, 0, 1 );
        colors[1] = glv::Color( 0, 1, 0 );
        colors[2] = glv::Color( 1, 0, 0 );

      }

      float       interpolant_values[3];
      glv::Color  colors[3];

      virtual std::pair< glv::Color, glv::Color> getBounds( float value )
      {
        float* lower_ptr = std::lower_bound( interpolant_values, interpolant_values+3, value );
        float* upper_ptr = std::upper_bound( interpolant_values, interpolant_values+3, value );

        return std::make_pair( colors[std::distance( interpolant_values, lower_ptr )-1], colors[std::distance( interpolant_values, upper_ptr )] );
      }

    };

    struct ColorInterpolator
    {

      ColorInterpolator( boost::shared_ptr< ColorMap > map = boost::make_shared< Jet >() ) : map(map)
      {

      }

      boost::shared_ptr<ColorMap> map;

      glv::Color operator()( double value );
    };

    void drawBitmapText(char *string,float x,float y,float z) ;

  } // Visualisers
} // L3
