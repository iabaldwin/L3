#include "CommandInterface.h"

static inline std::string& ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}


namespace L3
{
    CommandInterface::CommandInterface( L3::Visualisers::DatasetLayout* layout, boost::shared_ptr< L3::Container > container )
        : layout(layout), container(container)
    {
        expression.reset( new boost::regex("^_" ) );

        member_function_map.insert( std::make_pair( "_load", &CommandInterface::load ) );
        member_function_map.insert( std::make_pair( "_algo", &CommandInterface::algo) );
        member_function_map.insert( std::make_pair( "_quit", &CommandInterface::quit) );
        member_function_map.insert( std::make_pair( "_script", &CommandInterface::script) );
        member_function_map.insert( std::make_pair( "_?", &CommandInterface::print) );
        
        member_function_map.insert( std::make_pair( "_add_traj", &CommandInterface::addTrajectory) );
    }

    bool CommandInterface::match( const std::string& current )
    {
        return boost::regex_search( current, (*expression) );
    }

    std::pair< std::string, std::string > parseCommand( const std::string& command_and_arguments )
    {
        std::string::const_iterator command_start = std::find( command_and_arguments.begin(), command_and_arguments.end(), '_' );
        std::string::const_iterator arguments_start = std::find( command_and_arguments.begin(), command_and_arguments.end(), ' ' );

        if ( command_start == command_and_arguments.end()  )
            return std::make_pair( "", command_and_arguments );
        else
            return std::make_pair( std::string( command_start, arguments_start ), std::string( arguments_start, command_and_arguments.end() ) );
    }

    std::pair< bool, std::string > CommandInterface::execute(const std::string& command )
    {
        std::pair< std::string, std::string > command_arguments = parseCommand( command );

        std::map< std::string, command_interpreter >::iterator it = member_function_map.find( command_arguments.first ); 
        if ( it != member_function_map.end() )
        {
            command_interpreter interpreter = it->second;
            return (this->*interpreter)( command_arguments.second );
        }
        else
            return std::make_pair( false, "CI::No appropriate load command" );


        /*
         *Runner
         */
        //{

        //size_t pos = command.find( "run" );

        //if ( pos != std::string::npos )
        //{
        //if (!container)
        //return std::make_pair( retval, "L3::No associated container" );

        //if( !container->experience )
        //return std::make_pair( retval, "L3::No current experience" );

        //return std::make_pair( true, "L3::Running" );

        //}

        //}

        /*
         *  Scripting
         */
        //{
        //{
        //size_t pos = command.find( "script" );

        //if ( pos != std::string::npos )
        //{
        //std::string script_target = command.substr( pos+6, command.size() );

                //}

        //}

        /*
         *  Stop
         */
        //{
        //size_t pos = command.find( "stop" );

        //if ( pos != std::string::npos )
        //{
        ////std::cout << "Stop called" << std::endl;
        ////container->runner.stop();

        //return std::make_pair( true, "L3::Stopped" );
        //}

        //}

        /*
         *  Reset
         */
        //{
        //size_t pos = command.find( "reset" );

        //if ( pos != std::string::npos )
        //{
        //return std::make_pair( true, "L3::Reset" );
        //}
        //}
        
    }

    std::pair< bool, std::string> CommandInterface::load( const std::string& load_command )
    {
        std::string load_copy( load_command );
        ltrim(load_copy);

        if (!container)
            return std::make_pair( false, "CI::No associated container" );

        try
        {
            container->dataset.reset( new L3::Dataset( load_copy ) );

            if( !(container->dataset->validate() &&container->dataset->load() ) )
                return std::make_pair( false, "CI::Could not validate: " + load_copy ); 
        }
        catch( ... )
        {
            return std::make_pair( false, "L3::No such directory: " + load_copy ); 
        }

        try
        {
            //container->mission.reset( new L3::Configuration::Mission( *container->dataset ) );
            //container->runner.reset( new L3::DatasetRunner( container->dataset.get(), container->mission.get() ) );
            //container->runner->start();
            //layout->load( container->runner.get() );
        }
        catch( ... )
        {
            return std::make_pair( false, "L3::No such configuration: " + load_copy ); 
        }

        return std::make_pair( true, "L3::Loaded dataset \t\t<" + load_copy + ">" );
    }


    std::pair< bool, std::string> CommandInterface::algo( const std::string& load_command )
    {
        if (!container)
            return std::make_pair( false, "CI::No associated container: " + load_command); 

        //container->cost_function.reset( new L3::Estimator::KLCostFunction<double>() );
        //container->algorithm.reset( new L3::Estimator::IterativeDescent<double>( container->cost_function.get(), container->experience->experience_pyramid ) );

        return std::make_pair( true, "CI::Loaded< Algorithm >" );
    }

    std::pair< bool, std::string> CommandInterface::print( const std::string& load_command )
    {
        std::stringstream help;

        help << std::endl << "--------HELP-----------" << std::endl;;
      
        for( std::map<std::string, command_interpreter>::iterator it = member_function_map.begin();
                it != member_function_map.end();
                it++ )
            help << it->first << std::endl;

        help << "--------/HELP-----------" << std::endl;;
        return std::make_pair( true, help.str() );
    
    }

    std::pair< bool, std::string> CommandInterface::quit( const std::string& load_command )
    {
        glv::Application::quit();
    }

    std::pair< bool, std::string> CommandInterface::script( const std::string& load_command )
    {
        std::string script_target ( load_command );
        ltrim(script_target);

        std::ifstream script( script_target.c_str() );

        if ( !script.good() )
            return std::make_pair( false, "CI::Script <" + script_target + "> failed {No such file}" );

        std::list< std::string > script_history;

        std::string line;

        while( getline( script, line ) )
            script_history.push_back( line );

        script.close();

        for( std::list< std::string  >::iterator it = script_history.begin();
                it != script_history.end(); 
                it++ )
        {

            // Comment
            if( (*it)[0] == '#' )
                continue;  

            std::pair< bool, std::string > result = (this->execute( *it ));
            if( !result.first )
                return std::make_pair( false, "CI::Script <" + script_target + "> failed @ " + *it + "\n{ " + result.second + " }\n" );
        
        }

        return std::make_pair( true, "CI::Script <" + script_target + "> succeeded" );
    }

    std::pair< bool, std::string> CommandInterface::addTrajectory( const std::string& load_command )
    {
        if( !layout )
            return std::make_pair( false, "CI::No associated layout" );

        std::string dataset_target( load_command );

        ltrim(dataset_target);
        
        // Read the poses 
        boost::scoped_ptr <L3::IO::BinaryReader< L3::SE3 > > pose_reader( ( new L3::IO::BinaryReader<L3::SE3>() ) ) ;
        boost::scoped_ptr< L3::Dataset > dataset;
      
        try
        {
            dataset.reset( new L3::Dataset( dataset_target ) );

            if (!pose_reader->open( dataset->path() + "/OxTS.ins" ) )
                return std::make_pair( false, "CI::Failed to load <" + dataset_target + ">" );

            // Read all the poses
            pose_reader->read();

            boost::shared_ptr< std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > > poses( new std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > () );
            
            // And extract them
            pose_reader->extract( *poses );

            // Here, we add a new component::leaf with the poses
            // Who owns it?

            boost::shared_ptr< L3::Visualisers::PoseSequenceRenderer > sequence( new L3::Visualisers::PoseSequenceRenderer( poses ) );

            //(*layout->composite) << *sequence;

            layout->addExtra( dataset_target, sequence );
        }
        catch( L3::no_such_folder& except )
        {
            return std::make_pair( false, "CI::Failed to add <" + dataset_target + ">" + "[" + except.what() + "]"  );
        }
            
        return std::make_pair( true, "CI::Added <" + dataset_target + ">" );

    }



    std::string CommandInterface::getState()
    {
        
    }
}
