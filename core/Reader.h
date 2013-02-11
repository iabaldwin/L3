#ifndef L3_IO_READER_H
#define L3_IO_READER_H

#include <vector>
#include <iterator>
#include <fstream>
#include "Datatypes.h"

#include <boost/shared_ptr.hpp>

namespace L3
{
namespace IO
{

template <typename T>
struct Extractor
{
    //std::vector<boost::shared_ptr<T> > elements;
    std::vector<T*> elements;

    int counter; 
    
    std::vector<double> buffer;

    Extractor() : counter(0)
    {
        buffer.resize(T::NUM_ELEMENTS);
    }

    ~Extractor()
    {
        //Memory leak?
    }

    void operator()( double d )
    {
        buffer[counter] = d;
        if( ++counter % T::NUM_ELEMENTS == 0 ) 
        {
            counter = 0;
            elements.push_back( new T( buffer ) );
            //elements.push_back( boost::shared_ptr<T>( new T( buffer ) ) );
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

        size_t read()
        {
            std::copy( std::istream_iterator<double>( stream ),
                        std::istream_iterator<double>(),
                        std::back_inserter( raw ) );

            #ifdef DEBUG
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
        
        bool extract( std::vector<L3::Pose*>& poses )
        {
            // TODO: Pass in a reference to an external vector, avoid memory leak
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

        bool extract(  std::vector<L3::LMS151*>& scans )
        {
            Extractor<L3::LMS151> e;
            
            e = std::for_each( raw.begin(), raw.end(), e );

            if ( e.counter == 0 && e.elements.size() > 0 )
            {
                scans.assign( e.elements.begin(), e.elements.end() );
                return true;
            }
       
            return true;
        }

};

}
}

#endif
