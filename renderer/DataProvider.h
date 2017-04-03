#ifndef L3_DATA_PROVIDER
#define L3_DATA_PROVIDER

#include "L3.h"

template <typename T>
struct RandomDataGenerator
{
  std::vector<double>  data;

  T operator()()
  {
    data.clear();

    for ( int i=0; i < L3::Sizes<T>::elements-1; i++  )
    {
      data.push_back( random() % 100 );
    }

    return T(data);
  }
};

namespace L3
{
namespace Data
{

  template <typename T>
    struct Provider
    {

      Provider( unsigned int SIZE = 20 ) : size(SIZE)
      {

      }

      unsigned int size;
      std::vector< T > data;

      RandomDataGenerator<T> generator;

      std::vector< T > operator()()
      {
        if ( data.size() == size )
          data.erase( data.begin() );

        data.push_back( generator() );

        return data;
      }

    };
}
}

#endif
