#include "Dataset.h"


int main(int argc, char* argv[] )
{
    if ( argc <2 )
        return -1;

    boost::shared_ptr< L3::Dataset > dataset( new L3::Dataset( argv[1] ) );
    
    dataset->validate();
    dataset->load();
    std::cout << *dataset << std::endl;

    dataset.reset();
}
