#include "Interface.h"

int main()
{

    L3::Interface* interface = new L3::LuaInterface();

    interface->init();

    interface->execute( "print(\"Hello World\" )" );

    delete interface;


}
