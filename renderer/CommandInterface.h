#ifndef L3_COMMAND_INTERFACE_H
#define L3_COMMAND_INTERFACE_H

#include "Interface.h"
#include "Layouts.h"

namespace L3
{
    struct CommandInterface : Interface
    {
        CommandInterface( L3::Container* container ) : container(container)
        {

        }

        L3::Container* container; 
        
        std::pair< bool, std::string> execute( const std::string& );
        
        std::string getState();
    };

}

#endif

