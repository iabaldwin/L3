#ifndef L3_UTILS_H
#define L3_UTILS_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string.h>

#include <boost/filesystem.hpp>

#include "Datatypes.h"
#include "Definitions.h"
#include "Core.h"

namespace L3
{

namespace Utils
{

namespace Math
{
    double degreesToRadians( double degrees );
}

struct Accumulator
{
    Accumulator() : counter(0), x(0.0), y(0.0), z(0.0)
    {
    }

    int counter;
    double x, y, z;
    void operator()( std::pair< double, boost::shared_ptr<L3::Pose > > p )
    {
        x += p.second->x;
        y += p.second->y;
   
        counter++;
    }

    std::vector<double> centroid()
    {
        std::vector<double> res(2);
        res[0] = x/counter;
        res[1] = y/counter;

        return res;
    }
};


//template <typename Iterator>
//void localisePoseChain( Iterator begin, Iterator end, const L3::Configuration::Locale& l )
//{
  
    //Iterator current = begin;
    //while(  current != end )
    //{
        //current->second->x -= l.x;
        //current->second->y -= l.y;
        
        //current++;
    //}
//}

template <typename Iterator>
void localisePoseChainToMean( Iterator begin, Iterator end )
{
    // Average 
    Accumulator a;
    a = std::for_each( begin, end, a );
    std::vector<double> centroid = a.centroid();

    Iterator it = begin;

    while( it != end )
    {
        (*it).second->x -= centroid[0];
        (*it).second->y -= centroid[1];

        // Regenerate homogeneous - this is poor
        (*it).second->_update();

        it++;
    }

}

template <typename Iterator>
void localisePoseChainToMean( Iterator input_begin, Iterator input_end, Iterator output_begin )
{
    // Average 
    Accumulator a;
    a = std::for_each( input_begin, input_end, a );
    std::vector<double> centroid = a.centroid();

    Iterator current_input = input_begin;
    Iterator current_output = output_begin;

    while( current_input != input_end )
    {
        *current_output = *current_input;

        (*current_output).second->x -= centroid[0];
        (*current_output).second->y -= centroid[1];

        // Regenerate homogeneous - this is poor
        (*current_output).second->_update();

        current_input++; current_output++;
    }

}

/*
 *Search
 */
template <typename T1, typename T2 >
struct matcher
{

    matcher( std::vector< std::pair< double, boost::shared_ptr<T1> > >* ELEMENTS,
            std::vector< std::pair< double, boost::shared_ptr<T1> > >* MATCHED
            ) : elements( ELEMENTS ), matched(MATCHED)
    {

    }

    L3::Comparator< std::pair< double, boost::shared_ptr<T1> > > _comparator;
    
    // Search by time
    void operator()( double data )
    {

    }

    // Search by time
    void operator()( std::pair< double, boost::shared_ptr<T2> > data  )
    {
        typename std::vector< std::pair< double, boost::shared_ptr<T1> > >::iterator index = std::lower_bound( (*elements).begin(), 
                                                                                                        (*elements).end(), 
                                                                                                        data.first,
                                                                                                        _comparator );

        // TODO:
        // Is this ok? Check the time here, don't assume
        if ( index == elements->end() ) 
        {
            index--; 
            std::cout << index->first - data.first << std::endl; 
        }

        matched->push_back( *index );
    }

    std::vector< std::pair< double, boost::shared_ptr<T1> > >* elements; 
    std::vector< std::pair< double, boost::shared_ptr<T1> > >* matched; 

};

template <typename T1, typename T2>
struct threader : std::binary_function< std::pair< double, boost::shared_ptr<T1> >,
                                        std::pair< double, boost::shared_ptr<T2> >, 
                                        std::pair< boost::shared_ptr<T1>, boost::shared_ptr<T2> > >
{
    std::pair< boost::shared_ptr<T1>, boost::shared_ptr<T2> > operator()( std::pair< double, boost::shared_ptr<T1> > a, std::pair< double, boost::shared_ptr<T2> > b ) 
    {
        return std::make_pair( a.second, b.second );        
    }

    std::vector< std::pair< boost::shared_ptr<T1>, boost::shared_ptr<T2> > > threaded;
};

/*
 *Configuration tools
 */
template <typename T>
void CalibrationToEigen( const std::vector<T>& calib, Eigen::Matrix4f& eig )
{

    L3::SE3 pose( calib );
    eig = pose.getHomogeneous();
}

}
}

#endif
