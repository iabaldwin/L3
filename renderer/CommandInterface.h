#ifndef L3_COMMAND_INTERFACE_H
#define L3_COMMAND_INTERFACE_H

#include "Interface.h"
#include "Layouts.h"
#include "Container.h"

#include <map>

namespace L3
{
    struct CommandInterface : Interface
    {
        CommandInterface( L3::Visualisers::DatasetLayout* layout, boost::shared_ptr< L3::Container > container );

        boost::shared_ptr< L3::Container > container; 
        
        L3::Visualisers::DatasetLayout* layout;
      
        typedef std::pair< bool, std::string > (CommandInterface::*command_interpreter)( const std::string& );

        //std::map< std::string, command_interpreter > member_function_map;
        std::map< std::string, std::pair< command_interpreter, std::string> > member_function_map;

        std::pair< bool, std::string> execute( const std::string& );
  
        /*
         *  Member functions
         */
        bool match( const std::string& current );
        
        std::string getState();

        std::list< boost::filesystem::path > paths;

        /*
         *  Command functions
         */
        std::pair< bool, std::string > load( const std::string& command );
        std::pair< bool, std::string > experience( const std::string& command );
        std::pair< bool, std::string > estimate( const std::string& command );
        std::pair< bool, std::string > algo( const std::string& command );
        std::pair< bool, std::string > cost_function( const std::string& command );
        std::pair< bool, std::string > print( const std::string& command );
        std::pair< bool, std::string > quit( const std::string& command );
        std::pair< bool, std::string > script( const std::string& command );
        
        std::pair< bool, std::string > experience_density( const std::string& command );
        
        std::pair< bool, std::string > stop( const std::string& command );
        std::pair< bool, std::string > start( const std::string& command );
        
        std::pair< bool, std::string > addTrajectory( const std::string& command );
        std::pair< bool, std::string > removeTrajectory( const std::string& command );
        std::pair< bool, std::string > removeTrajectories( const std::string& command );
        std::pair< bool, std::string > clear( const std::string& command );
        std::pair< bool, std::string > history( const std::string& command );

        std::pair< bool, std::string > addPath( const std::string& command );
        std::pair< bool, std::string > printPath( const std::string& command );
    };

}

#endif

