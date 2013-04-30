#include <iostream>
#include <fstream>

#include "Layouts.h"

int main()
{
    // Create window
    glv::Window win(1400, 800, "Visualisation::Layout");
    
    // Populate layout
    L3::Visualisers::Layout layout(win);

    // Create glv root
    glv::GLV top;
    
    // Create run
    //layout.run( top );

}
