#ifndef L3_IO_H
#define L3_IO_H

#include <vector>
#include <iterator>
#include <fstream>
#include "Datatypes.h"

namespace L3
{
namespace IO
{

class Reader {

    public:

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

            return raw.size();
        }

        std::vector<double> raw;

    protected:

        std::ifstream stream;

    private:
};

template <typename T>
struct Extractor
{
    std::vector<T*> elements;

    int mod, counter; 
    
    std::vector<double> buffer;

    Extractor() : counter(0)
    {
        //std::cout << T::ELEMENTS << std::endl;
        buffer.resize(T::NUM_ELEMENTS);
    }

    ~Extractor()
    {
        //Memory leak
    }

    
    void operator()( double d )
    {
        buffer[counter] = d;
        if( ++counter % mod == 0 )
        {
            counter = 0;
            elements.push_back( new T( buffer ) );
        }
    }
};

class PoseReader : public Reader
{
    public:
        
        bool extractPoses( std::vector<L3::Pose*>& poses )
        {
            Extractor<L3::SE3> e;
       
            e = std::for_each( raw.begin(), raw.end(), e );

            if ( e.counter == 0 )
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

        bool extractLIDAR(  std::vector<L3::LIDAR*>& scans )
        {
            Extractor<L3::LIDAR> p;
            return true;
        }

};

}
}

#endif
