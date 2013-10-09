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

#include "Core.h"
#include "Timing.h"
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

    MaskPolicy< T > DEFAULT_MASK_POLICY;

    std::ifstream input_stream;
    
    typename std::deque< std::pair< double, boost::shared_ptr<T> > > window;
    typename std::deque< std::pair< double, boost::shared_ptr<T> > > temp;

    virtual ~SlidingWindow();

    virtual bool initialise();
    void run();
    void stop();

    bool update( double time );

    std::deque< std::pair< double, boost::shared_ptr<T> > > getWindow();

    virtual int read();

    void purge();

    bool good();

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

    bool initialise()
    {
        this->input_stream.open( this->target.c_str(), std::ios::binary ); 
    
#ifndef NDEBUG
        L3::Timing::SysTimer t;
        std::cout << "Binary:" << "Buffering...";
        t.begin();
#endif
        double duration = 0;

        while ( duration < SlidingWindow<T>::window_duration )
        {
            int entries_read = read();

            if ( entries_read != this->STACK_SIZE )
                // End of stream, this is all we have
                return false;

            duration = this->window.back().first - this->window.front().first;
            
            std::cout << entries_read << ":" << this->STACK_SIZE << ":" << duration << std::endl;
        }
#ifndef NDEBUG
        std::cout << this->window.size() << " entries read in " << t.elapsed() << "s" << std::endl;
#endif
        this->initialised = true;
   
        return this->initialised;
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

            tmp.push_back( L3::AbstractFactory<T>::produce( entry, &this->DEFAULT_MASK_POLICY ) );
        }

        this->mutex.lock();
        this->window.insert( this->window.end(), tmp.begin(), tmp.end() ); 
        this->mutex.unlock();
 
        return i;
    }
};

}

#endif
