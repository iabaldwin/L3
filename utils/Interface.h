#ifndef L3_LUA_INTERFACE_H
#define L3_LUA_INTERFACE_H
#include <iostream>

#include <boost/regex.hpp>

extern "C"
{
       #include "lua.h"
       #include "lauxlib.h"
       #include "lualib.h"
}

namespace L3
{

struct Interface
{
    boost::shared_ptr<boost::regex> expression;
        
    virtual std::pair< bool, std::string>  execute( const std::string& ) = 0;

    virtual std::string getState() = 0;
  
    virtual bool match( const std::string&  ) = 0;

    virtual ~Interface()
    {

    }

};

struct LuaInterface : Interface
{
    LuaInterface() 
    {
        /* initialize lua */
        state = lua_open();

        /* load lua libraries */
        luaL_openlibs(state);
   
        setPath();
   
        expression.reset( new boost::regex("^_" ) );
    }

    lua_State* state;

    bool match( const std::string& current )
    {
        return !boost::regex_search( current, (*expression) );

    }

    std::pair< bool, std::string> execute( const std::string& str ) 
    {
        bool result = luaL_dostring( state, str.c_str() ) == 0 ? true : false;
     
        std::pair< bool, std::string> retval;

        if( result )
        {
            retval.first = true;
            retval.second = "";
        }
        else
        {
            retval.first = false;
            retval.second = getState();
        }
       

        return retval;
    }

    std::string getState()
    {
        const char* msg = lua_tostring( state, 1 );
        lua_pop(state, 1);
        return std::string( msg );
    }

    void setPath();

    ~LuaInterface()
    {
        if( state )
            lua_close(state);
    }

};

}

#endif

