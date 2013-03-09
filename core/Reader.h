#ifndef L3_READER_H
#define L3_READER_H

#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>

#include <boost/shared_ptr.hpp>

#include "Datatypes.h"
#include "AbstractFactory.h"

namespace L3
{
namespace IO
{

template <typename T>
struct TextExtractor
{
    std::vector< std::pair< double, boost::shared_ptr<T> > > elements;

    void operator()( std::string& s )
    {
        double time;
        std::stringstream ss( s );
        ss >> time;

        std::string data_str = ss.str();

        elements.push_back( AbstractFactory<T>::produce( data_str ) );
    }

};

template <typename T>
struct BinaryExtractor
{
    std::vector< std::pair< double, boost::shared_ptr<T> > > elements;

    BinaryExtractor() : counter(0)
    {
        buffer.resize( L3::Sizes<T>::elements, 0 );
    }

    int counter, index;
    std::vector<double> buffer;

    void operator()( double d )
    {
        index = (counter % L3::Sizes<T>::elements );
        
        counter++;
   
        if ( index == L3::Sizes<T>::elements -1 ) 
        {
            elements.push_back( AbstractFactory<T>::produce( buffer ) );
        }
        else
            buffer[index] = d;
    }

};

/*
 *Readers
 */
class Reader
{

    public:

        virtual bool    open( const std::string& f ) = 0;
        virtual size_t  read() = 0;

        virtual ~Reader()
        {
            if (stream.is_open())
                stream.close();
        }

    protected:

        std::ifstream stream;
};


/*
 *Binary classes
 */
template <typename T>
class BinaryReader : public Reader
{
    public:

        bool open( const std::string& f )
        {
            stream.open( f.c_str(), std::ios::in | std::ios::binary );
            return stream.good();
        }

        size_t read()
        {
            std::copy( std::istreambuf_iterator<char>( stream.rdbuf() ),
                        std::istreambuf_iterator<char>( ),
                        std::back_inserter( bytes ) );

            return bytes.size();
        }

        bool extract( std::vector< std::pair< double, boost::shared_ptr<T> > >& poses ) 
        {
            // How many elements?
            size_t numels = bytes.size()/sizeof(double);
            
            double* ptr = reinterpret_cast<double*>(&bytes[0]);

            BinaryExtractor<T> e;
       
            e = std::for_each( ptr, ptr+numels, e );

            poses.assign( e.elements.begin(), e.elements.end() );
        
            return true;
        }

        std::vector<unsigned char> bytes;
};

/*
 *Text classes
 */
class TextReader : public Reader
{

    public:

        bool open( const std::string& f )
        {
            stream.open( f.c_str(), std::ios::in );
            return stream.good();
        }

        size_t bytes;

        virtual size_t read()
        {

            stream.seekg (0, std::ios::end);
            bytes = stream.tellg();
            stream.seekg (0, std::ios::beg); 

            std::string line;

            while ( std::getline( stream, line ) )
                raw.push_back( line );

            return raw.size();
        }

        std::vector<std::string> raw;
    
};


/*
 *File readers
 */
template <typename T>
class BulkReader : public TextReader
{
    public:
        
        bool extract( std::vector< std::pair< double, boost::shared_ptr<T> > >& poses )
        {
            TextExtractor<T> e;
       
            e = std::for_each( raw.begin(), raw.end(), e );

            if ( e.elements.size() > 0 )
            {
                poses.assign( e.elements.begin(), e.elements.end() );
                return true;
            }
            else
                return false;
        }

};

template <typename T>
class SequentialFileReader : public Reader
{
    public:
        
        size_t read()
        {
            if ( stream.good() )
            {
                std::getline( stream, current_line );
                return current_line.size();
            }
                
            else
                return 0;
        }

        std::string current_line;

        bool extract( std::pair< double, boost::shared_ptr<T> >& pose )
        {
            TextExtractor<T> e;
            //e = std::for_each( raw.begin(), raw.end(), e );
       
            return true;
        }

};




}
}

#endif
