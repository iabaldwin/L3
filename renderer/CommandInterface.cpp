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

        // This could be much cleaner
        //if ( command == "quit" )
        //{
            //glv::Application::quit();
            //return std::make_pair( true, "" ); 
        //}

            //size_t pos = command.find( "load::" );
            
            //if ( pos != std::string::npos )
            //{
                //std::cout << "LOAD" << std::endl; 
        
            //return std::make_pair( true, "" );
            //}
            
        //return std::make_pair( false, "" );

    }

    std::string CommandInterface::getState()
    {
        
    }
}
