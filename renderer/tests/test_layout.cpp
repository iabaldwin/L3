#include <iostream>
#include <fstream>

#include "Layouts.h"

int main()
{
    glv::Window win(1400, 800, "Visualisation::Layout");
    
    L3::Visualisers::Layout layout(win);

    glv::GLV top;
    
    layout.go( top );

}
