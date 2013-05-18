#include <iostream>
#include <fstream>

#include "Layouts.h"

int main()
{
    // Create window
    glv::Window win(1400, 800, "Visualisation::Layout");
    
    L3::Visualisers::Layout layout(win);
    
    // Create run
    layout.run();

}
