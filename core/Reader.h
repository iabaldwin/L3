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
struct Extractor
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

class Reader 
{

    public:
        
        virtual ~Reader()
        {
            if (stream.is_open())
                stream.close();
        }

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

            std::string line;

            while ( std::getline( stream, line ) )
                raw.push_back( line );

#ifndef NDEBUG
            std::cout << raw.size() << " elements read" << std::endl;
#endif

            return raw.size();
        }

        std::vector<std::string> raw;

    protected:

        std::ifstream stream;

    private:
};

template <typename T>
class FileReader : public Reader
{
    public:
        
        bool extract( std::vector< std::pair< double, boost::shared_ptr<T> > >& poses )
        {
            Extractor<T> e;
       
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

}
}

#endif
