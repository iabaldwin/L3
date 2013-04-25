#ifndef L3_LUA_INTERFACE_H
#define L3_LUA_INTERFACE_H
#include <iostream>

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

    virtual bool init() = 0;
    
    virtual bool execute( const std::string& ) = 0;

    virtual ~Interface()
    {

    }




};

struct LuaInterface : Interface
{

    bool init()
    {
        /* initialize lua */
        state = lua_open();

        /* load lua libraries */
        luaL_openlibs(state);
    }

    
    bool execute( const std::string& str ) 
    {
        //std::string doString = luaL_dostring( l, "print(\"Hello World\")" );
        //std::string doString = luaL_dostring( state, str.c_str() );
        return luaL_dostring( state, str.c_str() );
    }


    ~LuaInterface()
    {
        if( state )
            lua_close(state);
    }

    lua_State* state;

};

}

#endif

