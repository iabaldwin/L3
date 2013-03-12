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
#include "Poco/Mutex.h"

#include "Tools.h"
#include "Core.h"
#include "Definitions.h"
#include "AbstractFactory.h"

namespace L3
{

template <typename T>
struct SlidingWindow : Poco::Runnable, TemporalObserver
{

    SlidingWindow( const std::string& input, double t ) : 
        read_required(false),
        running(true),
        initialised(false),
        STACK_SIZE(10*100),
        window_duration(t),
        proximity(10.0),
        target(input) 
    {
    }
   

    Poco::Mutex mutex;
   
    bool    read_required;
    bool    running;
    bool    initialised;
    int     STACK_SIZE;
    double  window_duration;
    double  proximity;
    double  current_time;
    const   std::string& target;
    
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
        assert( initialised );
        
        current_time = time;

        mutex.lock();
        double diff = window.back().first - current_time;
        mutex.unlock();

        // Need more data?
        if (  diff < this->proximity ) 
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
        if ( !initialised )
            throw std::exception();

        while( running )
        {
            if ( read_required )
            {
                read_required = false;  
   
                read();
                purge();

                if ( !good() )
                    stop(); // Is the stream finished?
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

        while ( duration < window_duration )
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

        initialised = true;
    }

    void purge()
    {
        mutex.lock(); 
        //while( window.back().first - window.front().first > window_duration )
        while( current_time  - window.front().first > window_duration )
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
        required  = L3::Sizes<T>::elements;
        entry.resize( required );
    }

    int required;
    std::vector<double> entry; 

    void initialise()
    {
        this->input_stream.open( this->target.c_str(), std::ios::binary ); 
    
#ifndef NDEBUG
        L3::Tools::Timer t;
        std::cout << "Buffering...";
        t.begin();
#endif
        double duration = 0;

        while ( duration < SlidingWindow<T>::window_duration )
        {
            int entries_read = read();

            if ( entries_read != this->STACK_SIZE )
                // End of stream, this is all we have
                return;

            duration = this->window.back().first - this->window.front().first;
        }
#ifndef NDEBUG
        std::cout << this->window.size() << " entries read in " << t.end() << "s" << std::endl;
#endif
        this->initialised = true;
    }

    int read()
    {
        int i;

        
        typename std::deque< std::pair< double, boost::shared_ptr<T> > > tmp;
        
        for ( i=0; i< this->STACK_SIZE; i++ )
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
