#include "CommandInterface.h"

namespace L3
{
    
    CommandInterface::CommandInterface( L3::Container* container ) : container(container)
    {
        expression.reset( new boost::regex("^_" ) );
    }

    bool CommandInterface::match( const std::string& current )
    {
        return boost::regex_search( current, (*expression) );

    }

    std::pair< bool, std::string > CommandInterface::execute(const std::string& command )
    {
        bool retval = false;
        {

            size_t pos = command.find( "load" );

            if ( pos != std::string::npos )
            {
                if (!container)
                    return std::make_pair( retval, "L3::No associated container" );
                
                std::string dataset_target = command.substr( pos+4, command.size() );

                try
                {
                    container->dataset.reset( new L3::Dataset( dataset_target ) );
                    container->dataset->validate();
                    container->dataset->load();
                }
                catch( ... )
                {
                    return std::make_pair( false, "L3::No such directory: " + dataset_target ); 
                }

                return std::make_pair( true, "L3::Loaded<" + dataset_target + ">" );
            }
        }

        {
            size_t pos = command.find( "quit" );

            if ( pos != std::string::npos )
            {
                glv::Application::quit();
            }
        }


        return std::make_pair( retval, "L3::No such command: " + command );

    }

    std::string CommandInterface::getState()
    {
        
    }
}
