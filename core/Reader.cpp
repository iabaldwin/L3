#include "Reader.h"

namespace L3
{
namespace IO
{
    template <typename T>
        void BinaryExtractor<T>::operator()( double datum )
        {
            index = (counter % L3::Sizes<T>::elements );

            counter++;
                
            buffer[index] = datum;

            if ( index == L3::Sizes<T>::elements -1 ) 
                elements.push_back( AbstractFactory<T>::produce( buffer ) );
        }


}
}

template void L3::IO::BinaryExtractor<L3::SE3>::operator()(double);
template void L3::IO::BinaryExtractor<L3::LMS151>::operator()(double);
template void L3::IO::BinaryExtractor<L3::LHLV>::operator()(double);
