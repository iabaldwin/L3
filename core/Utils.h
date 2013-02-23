#ifndef L3_UTILS_H
#define L3_UTILS_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include "Datatypes.h"
#include "Definitions.h"
#include <libconfig.h++>

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

//std::ostream& operator<<( std::ostream& o, const SWATHE& s )
//{
    //SWATHE_ITERATOR_CONST it = s.begin();

    //while( it != s.end() )
    //{
        //std::cout << (*it).first << ":" << (*it).second << std::endl;
    //}

    //return o;
//}


namespace L3
{

namespace Utils
{

/*
 *Printers
 */

void printWindow( std::deque< std::pair< double, boost::shared_ptr< L3::SE3 > > >& window )
{
    std::deque< std::pair< double, boost::shared_ptr< L3::SE3 > > >::const_iterator it = window.begin();

    while ( it != window.end() )
    {
        std::cout << (*it).first << std::endl;
   
        it++;
    }
}

/*
 *Directory management
 */
boost::filesystem::path configurationDirectory( void )
{
 char * pPath;
    pPath = getenv ("HOME");
    
    if (pPath==NULL)
        return boost::filesystem::path();

    boost::filesystem::path p;
    p /= std::string( pPath );

    p /= "code";
    p /= "matlab";
    p /= "conf";

    if ( !boost::filesystem::exists(p) || !boost::filesystem::is_directory( p ) )
        return boost::filesystem::path();
    else
        return p;
   
};

struct Locale 
{
    std::string name;

    Locale( const std::string& target )
    {

        boost::filesystem::path p = L3::Utils::configurationDirectory();

        p /= target;

        try
        {
            libconfig::Config config;
        
            f = fopen( p.string().c_str(), "r"  );

            config.read( f );

            double result;
            if (config.lookupValue( "datum.X.lower", result) )
            {
                x = result;
            }

            if (config.lookupValue( "datum.Y.lower", result) )
            {
                y = result;
            }
        }  
        catch( ... )
        {

        }


    }

    FILE* f;
    float x,y,z;

};

struct BEGBROKE : Locale
{
    BEGBROKE() : Locale("begbroke_datum.config") 
    {
    }

};

struct WOODSTOCK : Locale
{
    WOODSTOCK() : Locale("woodstock_datum.config") 
    {
    }

};

void localisePoseChainToOrigin( POSE_SEQUENCE& poses )
{

    double origin_x = poses.front().second->x;
    double origin_y = poses.front().second->y;

    POSE_SEQUENCE_ITERATOR it = poses.begin();

    while( it != poses.end() )
    {
        (*it).second->x -= origin_x;
        (*it).second->y -= origin_y;

        (*it).second->_update();

        it++;
    }
}

template <typename Iterator>
void localisePoseChain( Iterator begin, Iterator end, const Locale& l )
{
  
    Iterator current = begin;
    while(  current != end )
    {
        current->second->x -= l.x;
        current->second->y -= l.y;
        
        current++;
    }
}

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


}

}

#endif
