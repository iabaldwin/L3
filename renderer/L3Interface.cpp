#include "L3Interface.h"
#include <iostream>

namespace L3
{    
    std::pair< bool, std::string> L3Interface::execute( const std::string& command )
    {
        // This could be much cleaner
        if ( command == "quit" )
        {
            glv::Application::quit();
            return std::make_pair( true, "" ); 
        }

        size_t pos = command.find( "load::" );

        if ( pos != std::string::npos )
        {
        }
    }
}
