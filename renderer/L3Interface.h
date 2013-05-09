#ifndef L3_L3_INTERFACE_H
#define L3_L3_INTERFACE_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Interface.h"

namespace L3
{
    struct Container;
    
    struct L3Interface  : Interface
    {
        L3Interface() 
        {

        }

        static const char marker = '_';
        
        // TODO: better hierarch here
        std::pair< bool, std::string>  execute( const std::string& command );
   
        std::string getState() 
        {

        }
    };

}
#endif

