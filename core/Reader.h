#ifndef L3_READER_H
#define L3_READER_H

#include <vector>
#include <iterator>
#include <fstream>

#include <boost/shared_ptr.hpp>

#include "Datatypes.h"

namespace L3
{
namespace IO
{


template <typename T>
struct Extractor
{
    std::vector< std::pair< double, T* > > elements;

    int counter; 
    
    std::vector<double> buffer;

    Extractor() : counter(0)
    {
        buffer.resize(T::NUM_ELEMENTS);
    }

    void operator()( double d )
    {
        buffer[counter] = d;
        if( ++counter % T::NUM_ELEMENTS == 0 ) 
        {
            counter = 0;
            T* element = new T( buffer );
            elements.push_back( std::make_pair( element->time, element ));
        }
    }

};

class Reader {

    public:

        virtual ~Reader();

        bool open( const std::string& f )
        {
            stream.open( f.c_str(), std::ios::in );
            return stream.good();
        }

        size_t bytes;

        size_t read()
        {

            stream.seekg (0, std::ios::end);
            bytes = stream.tellg();
            stream.seekg (0, std::ios::beg); 

            std::copy( std::istream_iterator<double>( stream ),
                        std::istream_iterator<double>(),
                        std::back_inserter( raw ) );

#ifndef NDEBUG
            std::cout << raw.size() << " elements read" << std::endl;
#endif

            return raw.size();
        }

        std::vector<double> raw;

    protected:

        std::ifstream stream;

    private:
};

class PoseReader : public Reader
{
    public:
        
        bool extract( std::vector< std::pair< double, L3::Pose*> >& poses )
        {
            // TODO: 
            // Pass in a reference to an external vector, avoid memory leak
            Extractor<L3::SE3> e;
       
            e = std::for_each( raw.begin(), raw.end(), e );

            if ( e.counter == 0 && (e.elements.size() > 0 ) )
            {
                poses.assign( e.elements.begin(), e.elements.end() );
                return true;
            }
            else
                return false;
        }

};

class LIDARReader : public Reader
{
    public:

        //bool extract(  std::vector<L3::LMS151*>& scans )
        //{
            //Extractor<L3::LMS151> e;
            
            //e = std::for_each( raw.begin(), raw.end(), e );

            //if ( e.counter == 0 && e.elements.size() > 0 )
            //{
                //scans.assign( e.elements.begin(), e.elements.end() );
                //return true;
            //}
       
            //return false;
        //}

};

class LHLVReader : public Reader
{

    public:

        //bool extract( std::vector<L3::LHLV*>& lhlv )
        //{
            //Extractor<L3::LHLV> e;
            
            //e = std::for_each( raw.begin(), raw.end(), e );

            //if ( e.counter == 0 && e.elements.size() > 0 )
            //{
                //lhlv.assign( e.elements.begin(), e.elements.end() );
                //return true;
            //}
       
            //return false;
        //}

};

}
}

#endif
