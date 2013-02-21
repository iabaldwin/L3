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

#include "Tools.h"
#include "Core.h"
#include "Definitions.h"
#include "AbstractFactory.h"

typedef char BYTE;

namespace L3
{

template <typename T>
struct SlidingWindow : Poco::Runnable, Observer
{

    SlidingWindow( const std::string& input, double t ) : 
        read_required(false),
        STACK_SIZE(100),
        target(input), 
        running(true), 
        time(t)
    {
    }
   

    Poco::Mutex mutex;
   
    const std::string& target;
    int STACK_SIZE;
    bool running, read_required;
    double time;
    
    std::ifstream input_stream;
    
    typename std::deque< std::pair< double, boost::shared_ptr<T> > > window;
    typename std::deque< std::pair< double, boost::shared_ptr<T> > > temp;

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

    bool update( double time )
    {
        double proximity = 20.0;
        
        mutex.lock();
        double diff = window.back().first - time;
        mutex.unlock();

        // Need more data?
        if ( ( diff > 0 ) && ( diff < proximity ) )
        {
            read_required = true;
        }
       
        return true;
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

    virtual int read()
    {
        int i;
        std::string line; 
        
        typename std::deque< std::pair< double, boost::shared_ptr<T> > > tmp;
                
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

            tmp.push_back( L3::AbstractFactory<T>::produce( line ) );
        }
       
        mutex.lock();
        window.insert( window.end(), tmp.begin(), tmp.end() ); 
        mutex.unlock();
        
        return i;
    }

    virtual void initialise()
    {
        input_stream.open(target.c_str()); 
        
#ifndef NDEBUG
        L3::Tools::Timer t;
        std::cout << "Buffering...";
        t.begin();
#endif
        double duration = 0;

        while ( duration < time )
        {
            int entries_read = read();

            if ( entries_read != STACK_SIZE )
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

};

template <typename T>
struct SlidingWindowBinary : SlidingWindow<T>
{
    SlidingWindowBinary( const std::string& input, double t ) 
        : SlidingWindow<T>( input, t )
    {
    }
 

    void initialise()
    {
        this->input_stream.open( this->target.c_str(), std::ios::binary ); 
    
#ifndef NDEBUG
        L3::Tools::Timer t;
        std::cout << "Buffering...";
        t.begin();
#endif
        double duration = 0;

        while ( duration < SlidingWindow<T>::time )
        {
            int entries_read = read();

            if ( entries_read != SlidingWindow<T>::STACK_SIZE )
            {
                // End of stream, this is all we have
                return;
            }

            duration = this->window.back().first - this->window.front().first;
        }
#ifndef NDEBUG
        std::cout << this->window.size() << " entries read in " << t.end() << "s" << std::endl;
#endif
    }


    int read()
    {
        int i;
        std::vector<double> entry; 

        int required  = L3::Sizes<T>::elements;

        entry.resize( required );
        
        typename std::deque< std::pair< double, boost::shared_ptr<T> > > tmp;
        
        for ( i=0; i< SlidingWindow<T>::STACK_SIZE; i++ )
        {
            // Is the stream good?
            if ( !this->good() )
                break;

            this->input_stream.read( (char*)(&entry[0]), required*sizeof(double) );

            tmp.push_back( L3::AbstractFactory<T>::produce( entry ) );
        }

        this->mutex.lock();
        this->window.insert( this->window.end(), tmp.begin(), tmp.end() ); 
        this->mutex.unlock();
 
        return i;
    }
};

}

#endif
