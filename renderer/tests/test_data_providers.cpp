#include <iostream>

#include "L3.h"
#include "DataProvider.h"

int main()
{

    {
    L3::Data::Provider<L3::SE3> provider;

    int i =100;

    while( i-->0 )
        std::cout << (provider()).size() << std::endl;

    }
    
    {
    L3::Data::Provider<L3::LHLV> provider;

    int i =100;

    while( i-->0 )
        std::cout << (provider()).size() << std::endl;

    }

}
