#include "LogCapture.h"

std::streamsize OutputSink::write(const char* s, std::streamsize n)
{

  if ( n>0)
  {
    if ( RING_BUFFER.size() > RING_SIZE )
      RING_BUFFER.pop_back();
    else
    {
      RING_BUFFER.push_front( std::string( s, s+n ) );
    }
  }

  std::clog <<  n << std::endl;

  return n;
}

namespace L3
{
  namespace Visualisers
  {

    void LogCapture::onDraw( glv::GLV& g )
    {
      std::stringstream ss; 

      for ( std::list<std::string>::iterator it= sink.RING_BUFFER.begin(); 
          it != sink.RING_BUFFER.end(); 
          it++ )
        ss <<  *it ;

      if( !ss.str().empty() )
        mText = ss.str();
    }
  }
}
