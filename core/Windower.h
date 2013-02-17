#ifndef L3_WINDOWER_H
#define L3_WINDOWER_H

#include <iostream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <queue>
#include <sstream>

#include <ctime>

#include "Poco/Runnable.h"
#include "Poco/Thread.h"
#include "Poco/Mutex.h"

#include "Core.h"

namespace L3
{

template <typename T>
struct SlidingWindow : Poco::Runnable, Observer
{

    SlidingWindow( const std::string& input ) : 
        running(true), 
        read_required(false)
    {
        // Open file
        input_stream.open(input.c_str()); 
        
        // Fill the buffer
        read();
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
        std::cout.precision(12);
        mutex.lock();

        double diff = time - window.back().first ;

        // Need more data?
        if ( ( diff > 0 ) && ( diff < 10 ) )
        {
            read_required = true;
        }
       
        mutex.unlock();
    }

    std::queue< std::pair< double, std::string > > window;
    //typename std::queue< std::pair< double, T* > > stack;
    bool running, read_required;

    std::queue< std::pair< double, std::string > > getWindow()
    {
        mutex.lock();
        return window;
        mutex.unlock();
    }

    void run()
    {
        while( running )
        {
            if ( read_required == true )
            {
                read_required = false;  // Disable read
       
                mutex.lock();
                read();                 // Push new data
                purge();                // Purge old data
                mutex.unlock();

                if ( !good())           // Is the stream finished?
                    stop();
            }
        }
    }

    const static int STACK_SIZE = 1000;
    void read()
    {
        static int counter = 0;
        std::string line; 
        for ( int i=0; i<STACK_SIZE; i++ )
        {
            std::getline( input_stream, line );
    
            std::stringstream ss( line );
            ss << std::noskipws;

            double time;
            ss >> time;

            window.push( std::make_pair( time, line ) );
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
        for ( int i=0; i< STACK_SIZE; i++ )
            window.pop();
    }

    bool good()
    {
        return input_stream.good() ? true : false; 
    };

    std::ifstream input_stream;
};

}

#endif
