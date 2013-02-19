#ifndef L3_WINDOWER_H
#define L3_WINDOWER_H

#include <iostream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <deque>
#include <sstream>

#include <ctime>

#include <boost/shared_ptr.hpp>

#include "Poco/Runnable.h"
#include "Poco/Thread.h"
#include "Poco/Mutex.h"

#include "Tools.h"
#include "Core.h"
#include "Definitions.h"
#include "AbstractFactory.h"

namespace L3
{

template <typename T>
struct SlidingWindow : Poco::Runnable, Observer
{

    SlidingWindow( const std::string& input, double t ) : 
        read_required(false),
        running(true), 
        time(t)
    {
        // Open file
        input_stream.open(input.c_str()); 
        
        // Fill the buffer
        initialise();
    }

    virtual ~SlidingWindow()
    {
        if ( input_stream.is_open() )
            input_stream.close();
        
        stop();
    }

    void stop()
    {
        running = false;
    }

    void update( double time )
    {
        mutex.lock();

        double diff = window.back().first - time;

        double proximity = 10.0;

        // Need more data?
        if ( ( diff > 0 ) && ( diff < proximity ) )
        {
            read_required = true;
        }
       
        mutex.unlock();
    }

    Poco::Mutex mutex;
    double time;
    bool running, read_required;
    typename std::deque< std::pair< double, boost::shared_ptr<T> > > window;
    typename std::deque< std::pair< double, boost::shared_ptr<T> > > temp;

    double getDuration()
    {
        double duration;
        mutex.lock();
        duration = window.back().first - window.front().first;
        mutex.unlock();

        return duration;
    }

    std::deque< std::pair< double, boost::shared_ptr<T> > > getWindow()
    {
        mutex.lock();
        temp = window;   
        mutex.unlock();
        
        return temp;
    }

    void run()
    {
        while( running )
        {
            if ( read_required )
            {
                read_required = false;  
                
                read();
                purge();

                if ( !good() )
                {   // Is the stream finished?
                    stop();
                }
            }
        }
    }

    const static int STACK_SIZE = 5*100;
    int read()
    {
        int i;
        std::string line; 
        
        mutex.lock();
        for ( i=0; i<STACK_SIZE; i++ )
        {
            // Is the stream good?
            if ( !good() )
                break;

            std::getline( input_stream, line );
           
            // Empty newlines?
            if ( line.size() == 0 )
                break;
           
            // Read data
            std::stringstream ss( line );
            ss << std::noskipws;

            // First entry is ALWAYS time
            double time;
            ss >> time;

            window.push_back( std::make_pair( time, L3::AbstractFactory<T>::fromString( line ) ) );
        }
        mutex.unlock();
        
        return i;
    }

    void initialise()
    {
#ifndef NDEBUG
        L3::Tools::Timer t;
        std::cout << "Buffering...";
        t.begin();
#endif
        double duration = 0;

        while ( duration < time )
        {
            int lines_read = read();

            if ( lines_read != STACK_SIZE )
            {
                // End of stream, this is all we have
                return;
            }

            duration = window.back().first - window.front().first;
        }
#ifndef NDEBUG
        std::cout << window.size() << " entries read in " << t.end() << "s" << std::endl;
#endif

    }

    void purge()
    {
        mutex.lock(); 
        while( window.back().first - window.front().first > time )
            window.pop_front();
        mutex.unlock();
    }

    bool good()
    {
        return input_stream.good() ? true : false; 
    };

    std::ifstream input_stream;
};

}

#endif
