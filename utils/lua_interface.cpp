#include <iostream>

extern "C"
{
       #include <lua.h>
       #include "lauxlib.h"
       #include "lualib.h"
}

lua_State* l;
int main()
{
    /* lua interpreter */
    int doString;

    /* initialize lua */
    l = lua_open();

    /* load lua libraries */
    luaL_openlibs(l);

    /* run the hello.lua script */
    //dofile = luaL_dofile(l, "hello.lua");

    //doString = luaL_dostring( l, "print(\"Hello World\")" );

    //doString =  luaL_dostring( l, "require( \"iuplua\" ) require( \"iupluacontrols\" ) require( \"iuplua_pplot51\"  ) " );
    doString =  luaL_dostring( l, "require( \"iuplua\" ); print \"HI\" " );
                                

//plot = iup.pplot{TITLE = \"A simple XY Plot\",
                                                    //MARGINBOTTOM\="35\",
                                                    //MARGINLEFT=\"35\",
                                                    //AXS_XLABEL=\"X\",
                                                    //AXS_YLABEL=\"Y\"
                                                    //}" );


    std::cout << doString << std::endl;

    /* cleanup Lua */
    lua_close(l);

    return 0;
}

