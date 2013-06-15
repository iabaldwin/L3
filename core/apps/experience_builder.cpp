#include <sstream>
#include "L3.h"

int main( int argc, char** argv )
{
    if ( argc < 4 ) 
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

    boost::shared_ptr< L3::ExperienceBuilder > experience_builder;

    switch( argc )
    {
    
        case 5:
            experience_builder.reset( new L3::ExperienceBuilder( dataset, atof( argv[2]), atof( argv[3]), atof( argv[4])  ) );
            break;
           
        case 6:
            experience_builder.reset( new L3::ExperienceBuilder( dataset, atof( argv[2]), atof( argv[3]), atof( argv[4]), atof( argv[5] )  ) );
            break;

        default:
            experience_builder.reset( new L3::ExperienceBuilder( dataset, atof( argv[2]), atof( argv[3]) ) );
            break;

    };

}
