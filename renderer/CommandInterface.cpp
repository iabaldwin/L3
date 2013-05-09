#include "CommandInterface.h"

namespace L3
{

    std::pair< bool, std::string > CommandInterface::execute(const std::string& command )
    {
        size_t pos = command.find( "load::" );

        bool retval = false;

        if ( pos != std::string::npos )
        {
            std::string dataset_target = command.substr( pos+6, command.size() );
            std::cout << "LOAD: " << dataset_target << std::endl;


            try
            {
                container->dataset.reset( new L3::Dataset( dataset_target ) );
                container->dataset->validate();
                container->dataset->load();
            }
            catch( ... )
            {
                return std::make_pair( false, "No such directory: " + dataset_target ); 
            }

            return std::make_pair( true, "" );
        }

        return std::make_pair( false, "" );

    }

    std::string CommandInterface::getState()
    {
        
    }
}
