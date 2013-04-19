#include <iostream>

#include "L3.h"
#include "DataProvider.h"

const char* demangle(const char* name)
{
    char buf[1024];
    size_t size =1024;
    int status;
    char* res = abi::__cxa_demangle (name,
            buf,
            &size,
            &status);
    return res;
}

int main()
{

    {
        L3::Data::Provider<L3::SE3> provider;

        int i =100;

        while( i-->0 )
            std::cout << (provider()).size() << " elements of " << demangle( typeid( provider ).name() ) << std::endl;

    }

    {
        L3::Data::Provider<L3::LHLV> provider;

        int i =100;

        while( i-->0 )
            std::cout << (provider()).size() << " elements of " << demangle( typeid( provider ).name() ) << std::endl;

    }

}
