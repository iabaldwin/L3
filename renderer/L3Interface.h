#ifndef L3_L3_INTERFACE_H
#define L3_L3_INTERFACE_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>


namespace L3
{

struct L3Interface  : Interface
{

    L3Interface()
    {



    }
    
    bool execute( const std::string& command )
    {
        // This could be much cleaner
        if ( command == "quit" )
            glv::Application::quit();
    }

};

}
#endif

