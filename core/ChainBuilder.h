#ifndef L3_CHAIN_BUILDER_H
#define L3_CHAIN_BUILDER_H

#include <cmath>
#include <numeric>
#include <iterator>

#include "Datatypes.h"
#include "Definitions.h"

typedef std::pair< double, boost::shared_ptr<L3::LHLV> > RECORD;

namespace L3
{

template <typename InputIterator, typename OutputIterator >
void partialAccumulate( InputIterator begin, InputIterator end, OutputIterator output )
{
    typedef typename std::iterator_traits<OutputIterator>::value_type VAR;

    InputIterator current_in = begin;
    
    // Initialise
    *output++ = VAR(current_in++->second->data );

    while ( current_in != end )
    {
        *output = VAR(current_in++->second->data ) + *(output-1);
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
        element<T> tmp;
        std::transform( data.begin(), data.end(), s.data.begin(), std::back_inserter( tmp.data ), std::plus<T>() );
        return tmp;
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
    POSE_SEQUENCE build( std::deque< RECORD > data )
    {
        std::vector< L3::element<double> > trajectory( data.size() );
   
        L3::partialAccumulate( data.begin(), data.end(), trajectory.begin() );

        std::copy( trajectory.begin(), 
                trajectory.end(),
                std::ostream_iterator<L3::element<double> >( std::cout, " " ));
        std::cout << std::endl;
    }
};

}

#endif
