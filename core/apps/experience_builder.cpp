#include <sstream>
#include "L3.h"

int main( int argc, char** argv )
{
    if ( argc != 4 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <char::dataset> <float::start_time> <float::end_time>" << std::endl;
        exit(-1);
    }

    char* dataset_directory = argv[1];
 
    /*
     *  L3
     */
    
    L3::Dataset dataset( dataset_directory );
    std::cout << dataset << std::endl;

    std::stringstream ss;

    L3::ExperienceBuilder experience_builder( dataset, atoi( argv[2]), atoi( argv[3]) );
}
