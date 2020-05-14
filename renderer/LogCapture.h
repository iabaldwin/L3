#pragma once

#include <iostream>
#include <sstream>
#include <list>

#include <boost/next_prior.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/stream_buffer.hpp>

#include <glv.h>

class OutputSink : public boost::iostreams::sink
{
  public:

    OutputSink( unsigned int RING_SIZE = 100 ) : RING_SIZE(RING_SIZE)
  {
  }

    int RING_SIZE;
    std::list < std::string >::iterator RING_PTR;
    std::list < std::string > RING_BUFFER;

    std::streamsize write(const char* s, std::streamsize n);

};

namespace L3
{
  namespace Visualisers
  {

    struct LogCapture : glv::TextView
    {
      LogCapture() : glv::TextView( glv::Rect( 600,300 ) )
      {
        sb.open(sink);
        previous_buffer = std::cout.rdbuf(&sb);
      }

      ~LogCapture()
      {
        std::cout.rdbuf(previous_buffer);
      }

      OutputSink sink;

      boost::iostreams::stream_buffer<OutputSink> sb;

      std::streambuf* previous_buffer;

      void onDraw( glv::GLV& g );
    };
  } // Visualisers
} // L3
