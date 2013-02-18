#ifndef L3_WINDOWER_H
#define L3_WINDOWER_H

#include <iostream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <deque>
#include <sstream>

#include <ctime>

#include "Poco/Runnable.h"
#include "Poco/Thread.h"
#include "Poco/Mutex.h"

#include "Core.h"
#include "Definitions.h"

namespace L3
{

template <typename T>
struct SlidingWindow : Poco::Runnable, Observer
{

    SlidingWindow( const std::string& input, double t ) : 
        running(true), 
        read_required(false),
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

    Poco::Mutex mutex;

    void stop()
    {
        running = false;
    }

    void update( double time )
    {
        mutex.lock();

        double diff = window.back().first - time;

        // Need more data?
        if ( ( diff > 0 ) && ( diff < 10 ) )
        {
            read_required = true;
        }
       
        mutex.unlock();
    }

    double time;
    WINDOW window;
    bool running, read_required;

    WINDOW getWindow()
    {
        WINDOW temp;
        
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
                read_required = false;  // Disable subsequent read
                
                mutex.lock();
                read();                 // Push new data
                purge();                // Purge old data
                mutex.unlock();

                if ( !good() )
                {   // Is the stream finished?
                    stop();
                }
            }
        }
    }

    const static int STACK_SIZE = 100;
    int read()
    {
        static int counter = 0;
        std::string line; 
        int i;
        for ( i=0; i<STACK_SIZE; i++ )
        {
            if ( !good() )
                break;

            std::getline( input_stream, line );
            std::stringstream ss( line );
            ss << std::noskipws;

            double time;
            ss >> time;

            window.push_back( std::make_pair( time, line ) );
        }
   
        return i;
    }

    void initialise()
    {
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
    }


    size_t size()
    {
        size_t s;
        mutex.lock();
        s = window.size(); 
        mutex.unlock();
        return s;
    }

    void purge()
    {
        //WINDOW_ITERATOR current = window.begin();
    
        //double diff = window.back().first - (*current).first;

        //std::cout << diff << std::endl;
    }

    bool good()
    {
        return input_stream.good() ? true : false; 
    };

    std::ifstream input_stream;
};

}

#endif
