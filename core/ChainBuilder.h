#ifndef L3_CHAIN_BUILDER_H
#define L3_CHAIN_BUILDER_H

#include <cmath>
#include <numeric>
#include <iterator>

#include "Datatypes.h"
#include "Definitions.h"

namespace L3
{

typedef std::pair< double, boost::shared_ptr<L3::LHLV> > RECORD;

std::ostream& operator<<( std::ostream& o, const RECORD& r ) 
{
    o<< r.first << ":";

    r.second->print(o);

    return o;
}

template <typename InputIterator, typename OutputIterator >
void partialAccumulate( InputIterator begin, InputIterator end, OutputIterator output )
{
    typedef typename std::iterator_traits<OutputIterator>::value_type VAR;

    InputIterator current_in = begin;

    double dt = 0;

    double current_time = current_in->first;
    
    // Initialise
    *output++ = (VAR(current_in++->second->data )*dt);
    
    while ( current_in != end )
    {
        dt = current_in->first - current_time;
      
        current_time = current_in->first;

        //*output = VAR(current_in++->second->data ) + *(output-1);
        VAR tmp = (VAR(current_in++->second->data )*dt) + *(output-1);
        *output = tmp;
        output++;
    }
}

template <typename T>
struct element
{
    element() 
    {
    }

    element( std::vector<T> DATA ) : data(DATA)
    {
    }

    std::vector<T> data;

    element<T> operator+( const element<T>& s ) const
    {

        /*
         *  Note: 
         *      Here we are adding everything, 
         *      but that doesn't make sense for some fields
         */

        element<T> tmp;
        std::transform( data.begin(), data.end(), s.data.begin(), std::back_inserter( tmp.data ), std::plus<T>() );
        return tmp;
    }
 
    element<T>& operator*( const T t ) 
    {
        std::transform( data.begin(), data.end(), data.begin(), std::bind2nd( std::multiplies<T>(), t ) );
        return *this;
    }
    
};

template <typename T>
std::ostream& operator<<( std::ostream& in , const element<T> & el )
{
    std::copy( el.data.begin(), 
            el.data.end(), 
            std::ostream_iterator<T>( in, " " ) );

    in << std::endl;

    return in; 
}


struct ChainBuilder
{
    L3::Tools::Timer t;
    POSE_SEQUENCE build( std::deque< RECORD > data )
    {
        std::vector< L3::element<double> > trajectory( data.size() );

        // Accumulate the data
        L3::partialAccumulate( data.begin(), data.end(), trajectory.begin() );

        std::copy( data.begin(), 
                    data.end(),
                    std::ostream_iterator< RECORD >( std::cout, " " ));
        std::cout << std::endl;

        std::copy( trajectory.begin(), 
                    trajectory.end(),
                    std::ostream_iterator<L3::element<double> >( std::cout, " " ));
        std::cout << std::endl;

        exit(1);

        return POSE_SEQUENCE();
    }
};

}

#endif
