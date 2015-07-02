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
//#include "Poco/Mutex.h"
#include <mutex>

#include "Core.h"
#include "Timing.h"
#include "Definitions.h"
#include "AbstractFactory.h"

#define _stack_size 100

namespace L3
{

template <typename T>
struct SlidingWindow : Poco::Runnable, TemporalObserver
{

    SlidingWindow( const std::string& input, double t ) : 
        read_required(false),
        running(true),
        initialised(false),
        STACK_SIZE(_stack_size),
        window_duration(t),
        proximity(10.0),
        target(input) 
    {
    }
   

    std::mutex  mutex;

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

    bool initialise();

    int read();
};

}

#endif
