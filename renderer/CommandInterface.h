#ifndef L3_COMMAND_INTERFACE_H
#define L3_COMMAND_INTERFACE_H

#include "Interface.h"
#include "Layouts.h"
#include "Container.h"

namespace L3
{
    struct CommandInterface : Interface
    {
        CommandInterface( L3::Visualisers::EstimatorLayout* layout );

        boost::shared_ptr< L3::Container > container; 
        L3::Visualisers::EstimatorLayout* layout;
        
        std::pair< bool, std::string> execute( const std::string& );
    
        bool match( const std::string& current );
        
        std::string getState();
    
    };

}

#endif

