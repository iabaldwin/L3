#include <iostream>

#include "L3.h"
#include "Misc.h"

int main()
{

    // Validate all the datsets
    std::list <std::string> datasets = L3::Misc::getDatasetConfigurations();

    for( std::list<std::string>::iterator it = datasets.begin(); it!= datasets.end(); it++ )
    {
        try
        {
        L3::Configuration::Mission mission( *it );
        std::cout << mission << std::endl;
        }
        catch( ... )
        {
            std::cerr << "Error: " << *it << std::endl;
        }
    }
}