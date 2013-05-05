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

    virtual bool execute( const std::string& ) = 0;

    virtual const char* get_state()
    {

    }
 
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
    }

    
    bool execute( const std::string& str ) 
    {
        //std::string doString = luaL_dostring( l, "print(\"Hello World\")" );
        //std::string doString = luaL_dostring( state, str.c_str() );
        return luaL_dostring( state, str.c_str() );
    }

    const char* get_state()
    {
          //const char *msg = lua_tostring(L, 1);
   
        return lua_tostring( state, 1 );
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

