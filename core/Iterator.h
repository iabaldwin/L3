#include <iostream>
#include <fstream>

class Iterator
{

    public:
    
        Iterator()
        {

        }


        bool open( const std::string filename )
        {
            std::ifstream input(filename.c_str() );
       

            return input.good();
        }

};


