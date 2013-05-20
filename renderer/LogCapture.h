#ifndef L3_VISUALISERS_LOG_CAPTURE_H
#define L3_VISUALISERS_LOG_CAPTURE_H

#include <iostream>
#include <sstream>
#include <list>

#include <boost/iostreams/concepts.hpp> 
#include <boost/iostreams/stream_buffer.hpp>

#include <GLV/glv.h>
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
            LogCapture()
            {
                //sb.open(OutputSink());
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


}
}


#endif
