#pragma once

#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>

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

      BinaryExtractor( MaskPolicy<T> policy = MaskPolicy<T>() ) : counter(0)
      {
        buffer.resize( L3::Sizes<T>::elements, 0 );
      }

      int counter, index;
      std::vector<double> buffer;

      void operator()( double d );
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

      virtual ~BinaryReader()
      {
      }

      virtual bool open( const std::string& f )
      {
        stream.open( f.c_str(), std::ios::in | std::ios::binary );
        return stream.good();
      }

      virtual size_t read()
      {
        std::copy( std::istreambuf_iterator<char>( stream.rdbuf() ),
            std::istreambuf_iterator<char>( ),
            std::back_inserter( bytes ) );

        return bytes.size();
      }

      virtual bool extract( std::vector< std::pair< double, boost::shared_ptr<T> > >& poses, MaskPolicy<T> policy = MaskPolicy<T>() ) 
      {
        // How many elements?
        size_t numels = bytes.size()/sizeof(double);

        double* ptr = reinterpret_cast<double*>(&bytes[0]);

        BinaryExtractor<T> extractor( policy );

        // Extract
        extractor = std::for_each( ptr, ptr+numels, extractor );

        poses.assign( extractor.elements.begin(), extractor.elements.end() );

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
   * Sequential readers
   */
  template <typename T>
    class SequentialBinaryReader : public BinaryReader<T>
  {
    public:

      size_t read()
      {
        // Allocate 
        this->bytes.resize(L3::Sizes<T>::elements*sizeof(double) );

        // Read
        if ( this->stream.good() )
        {
          this->stream.read( (char*)&(this->bytes[0]), L3::Sizes<T>::elements*sizeof(double) );
          return this->stream.gcount();
        }
        else {
          return 0;
        }
      }

  };
}
}
