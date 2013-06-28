#include "CommandInterface.h"

static inline std::string& ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

std::list< std::string > added_trajectories;

namespace L3
{
    CommandInterface::CommandInterface( L3::Visualisers::DatasetLayout* layout, boost::shared_ptr< L3::Container > container )
        : layout(layout), container(container)
    {
        expression.reset( new boost::regex("^_" ) );

        member_function_map.insert( std::make_pair( "_load",            std::make_pair( &CommandInterface::load,                "Load a dataset, for viewing" ) ));
        member_function_map.insert( std::make_pair( "_exp",             std::make_pair( &CommandInterface::experience,          "Load an experience" ) ));
        member_function_map.insert( std::make_pair( "_estimate",        std::make_pair( &CommandInterface::estimate,            "Load a dataset, for estimation" ) ) );
        member_function_map.insert( std::make_pair( "_algo",            std::make_pair( &CommandInterface::algo,                "Set current estimation algorithm" ) ) );
        member_function_map.insert( std::make_pair( "_cost",            std::make_pair( &CommandInterface::cost_function,       "Set current estimation algorithm" ) ) );
        member_function_map.insert( std::make_pair( "_quit",            std::make_pair( &CommandInterface::quit,                "Leave" ) ) );
        member_function_map.insert( std::make_pair( "_script",          std::make_pair( &CommandInterface::script,              "Execute a script" ) ) );
        member_function_map.insert( std::make_pair( "_?",               std::make_pair( &CommandInterface::print,               "Print this help" ) ) );
        member_function_map.insert( std::make_pair( "_history",         std::make_pair( &CommandInterface::history,             "Print (successful) history" ) ) );
       
        member_function_map.insert( std::make_pair( "_exp_density",     std::make_pair( &CommandInterface::experience_density,  "Set experience density" ) ) );

        member_function_map.insert( std::make_pair( "_stop",            std::make_pair( &CommandInterface::stop,                "Stop running" ) ) );
        member_function_map.insert( std::make_pair( "_start",           std::make_pair( &CommandInterface::start,               "Start running" ) ) );
        member_function_map.insert( std::make_pair( "_mode",            std::make_pair( &CommandInterface::runMode,             "Change run mode" ) ) );
        
        member_function_map.insert( std::make_pair( "_add_traj",        std::make_pair( &CommandInterface::addTrajectory,       "Add a visual trajectory" ) ) );
        member_function_map.insert( std::make_pair( "_add_path",        std::make_pair( &CommandInterface::addPath,             "Add a search path") ) );
        member_function_map.insert( std::make_pair( "_print_path",      std::make_pair( &CommandInterface::printPath,           "Print the search path") ) );
        member_function_map.insert( std::make_pair( "_remove_traj",     std::make_pair( &CommandInterface::removeTrajectory,    "Remove a visual trajectory") ) );
        member_function_map.insert( std::make_pair( "_remove_all_traj", std::make_pair( &CommandInterface::removeTrajectories,  "Remove all trajectories") ) );
        member_function_map.insert( std::make_pair( "_clear",           std::make_pair( &CommandInterface::clear,               "Clear current session") ) );
   
        addPath( "/Users/ian/code/L3/scripts/" );
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

    std::vector< std::string > parseArgs( const std::string& command )
    {
        std::string::const_iterator ptr = command.begin();
        std::string::const_iterator break_ptr;

        std::vector< std::string > args;

        while( ptr != command.end() )
        {
            ptr = std::find( ptr, command.end(), '-' );
       
            if( ptr != command.end() )
            {
                break_ptr = std::find( ptr+1, command.end(), ' ' );
               
                std::string arg( ptr, break_ptr );
              
                if( arg.size() <= 3 )
                {
                    args.push_back( arg );
                }
                ptr++; 
            }
        }

        return args;
    }

    std::pair< bool, std::string > CommandInterface::execute(const std::string& command )
    {
        std::pair< std::string, std::string > command_arguments = parseCommand( command );

        parseArgs( command );

        if ( command_arguments.first.size() == 0 )
            return std::make_pair( false, "CI::Parse error" );

        std::map< std::string, std::pair< command_interpreter, std::string>  >::iterator it = member_function_map.find( command_arguments.first ); 
        if ( it != member_function_map.end() )
        {
            command_interpreter interpreter = it->second.first;
            return (this->*interpreter)( command_arguments.second );
        }
        else
            return std::make_pair( false, "CI::No appropriate command [" + command + "]"  );
    }

    std::pair< bool, std::string> CommandInterface::load( const std::string& load_command )
    {
        if (!container)
            return std::make_pair( false, "CI::No associated container" );

        std::string load_copy( load_command );
        ltrim(load_copy);
      
        try
        {
            container->dataset.reset( new L3::Dataset( load_copy ) );

            if( !(container->dataset->validate() &&container->dataset->load() ) )
                return std::make_pair( false, "CI::Could not validate: " + load_copy ); 
        }
        catch( ... )
        {
            return std::make_pair( false, "CI::No such directory: " + load_copy ); 
        }

        container->mission.reset( new L3::Configuration::Mission( *container->dataset ) );
        container->runner.reset( new L3::DatasetRunner( container->dataset.get(), container->mission.get() ) );
        layout->load( container->runner.get() );

        return std::make_pair( true, "CI::Loaded dataset \t\t<" + load_copy + ">" );
    }

    std::pair< bool, std::string> CommandInterface::estimate( const std::string& load_command )
    {
        if (!container)
            return std::make_pair( false, "CI::No associated container" );

        std::string load_copy( load_command );
        ltrim(load_copy);
        
        try
        {
            container->dataset.reset( new L3::Dataset( load_copy ) );

            if( !(container->dataset->validate() &&container->dataset->load() ) )
                return std::make_pair( false, "CI::Could not validate: " + load_copy ); 
        }
        catch( ... )
        {
            return std::make_pair( false, "CI::No such directory: " + load_copy ); 
        }

        // Reseat algorithm
        boost::shared_ptr< L3::Estimator::Algorithm<double> > algorithm = dynamic_cast<L3::EstimatorRunner*>(container->runner.get())->algorithm;

        // Reset
        container->mission.reset( new L3::Configuration::Mission( *container->dataset ) );
        container->runner.reset( new L3::EstimatorRunner( container->dataset.get(), container->mission.get(), container->experience.get() ) );
        dynamic_cast< L3::EstimatorRunner* >(container->runner.get())->setAlgorithm ( algorithm );
        container->runner->start();
        layout->load( container->runner.get() );

        return std::make_pair( true, "CI::Loaded dataset \t\t<" + load_copy + ">" );
    
    }
    
    std::pair< bool, std::string> CommandInterface::experience( const std::string& load_command )
    {
        if (!container)
            return std::make_pair( false, "CI::No associated container: " + load_command); 

        try
        {
            std::string load_copy( load_command );
            ltrim(load_copy);
        
            L3::Dataset experience_dataset( load_copy );
            L3::ExperienceLoader experience_loader( experience_dataset );
           //boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

            container->experience = experience_loader.experience;
            
            return std::make_pair( true, "CI::Experience <: " + load_command + "> loaded" ); 

        }
        catch ( ... )
        {
            return std::make_pair( false, "CI::Failed to load experience <: " + load_command + "> loaded" ); 
        }
    }

    /*
     *  Algorithm selector
     */
    std::pair< bool, std::string> CommandInterface::algo( const std::string& load_command )
    {
        if (!container)
            return std::make_pair( false, "CI::No associated container: " + load_command); 

        if (!layout)
            return std::make_pair( false, "CI::No associated layout : " + load_command); 

        if( boost::shared_ptr< EstimatorRunner > runner = boost::dynamic_pointer_cast< EstimatorRunner >( container->runner) )
        {
            L3::WriteLock algorithm_lock( runner->mutex );
            
            boost::shared_ptr< L3::Estimator::CostFunction<double > > cost_function = runner->algorithm->cost_function; 

            std::string algo_target( load_command );
            ltrim( algo_target );

            boost::shared_ptr< L3::Estimator::Algorithm<double> > algo 
                = L3::Estimator::AlgorithmFactory<double>::produce( algo_target, cost_function, container->experience->experience_pyramid, runner );

            if( !algo )
                return std::make_pair( false, "CI::Failed to load <" + algo_target + ">" );

            //Assign cost function
            algo->cost_function = cost_function;

            //Set algorithm
            runner->setAlgorithm( algo );
    
            // Reset pose
            *runner->current = runner->oracle->operator()(); 
        
            // Reset pose
            //L3::SE3 pose_estimate = runner->oracle->operator()();
            //L3::SE3 closest_pose = runner->experience->getClosestPose( pose_estimate );
            //closest_pose.Q( pose_estimate.Q() );
            //*runner->current = closest_pose;

            // Associate algorithm, visually
            dynamic_cast< L3::Visualisers::EstimatorLayout* >( layout )->algorithm( algo );

            algorithm_lock.unlock();

            return std::make_pair( true, "CI::Loaded< " + algo_target + ">" );
        
        }

        return std::make_pair( false, "CI::Failed to load< Algorithm >" );
    }

  
    /*
     *  Cost-function selector
     */
    std::pair< bool, std::string> CommandInterface::cost_function( const std::string& load_command )
    {
        if (!container)
            return std::make_pair( false, "CI::No associated container: " + load_command); 

        if (!layout)
            return std::make_pair( false, "CI::No associated layout : " + load_command); 

            
        std::string cost_function_target( load_command );
        ltrim( cost_function_target );

        if( boost::shared_ptr< EstimatorRunner > runner = boost::dynamic_pointer_cast< EstimatorRunner >( container->runner) )
        {
            boost::shared_ptr< L3::Estimator::CostFunction<double> > cost_function = L3::Estimator::CostFactory<double>::produce( cost_function_target );

            if( !cost_function )
                return std::make_pair( false, "CI::Failed to load <" + cost_function_target + ">" );

            L3::WriteLock algorithm_lock( runner->mutex );

            //Assign cost function
            runner->algorithm->cost_function = cost_function;

            algorithm_lock.unlock();

            return std::make_pair( true, "CI::Loaded< " + cost_function_target + " >" );
        
        }

        return std::make_pair( false, "CI::Failed to load< " + cost_function_target + " >" );
    }

    std::pair< bool, std::string > CommandInterface::experience_density( const std::string& command )
    {
        if (!container)
            return std::make_pair( false, "CI::No associated container: " + command ); 

        L3::WriteLock lock( container->experience->mutex );

        if ( boost::shared_ptr< L3::HistogramUniformDistance<double> > ptr = 
                boost::dynamic_pointer_cast< L3::HistogramUniformDistance<double> >( container->experience->experience_pyramid->histograms.front() ) )
        {
            ptr->bins_per_metre = 0.1;
        }

        lock.unlock();

        return std::make_pair( true, "CI::Density" ); 


    }

    std::pair< bool, std::string> CommandInterface::print( const std::string& load_command )
    {
        std::stringstream help;

        help << std::setfill(' ') << std::setw(20);

        help << "\n--------HELP-----------\n";
      
        for( std::map<std::string, std::pair< command_interpreter, std::string > >::iterator it = member_function_map.begin();
                it != member_function_map.end();
                it++ )
            help << it->first << "\t:\t" << it->second.second << std::endl;

        help << "--------/HELP-----------" << std::endl;;
        return std::make_pair( true, help.str() );
    
    }

    std::pair< bool, std::string> CommandInterface::quit( const std::string& load_command )
    {
        glv::Application::quit();
   
        return std::make_pair( false, ""  );    // We never get here, just to satisfy the compiler
    }

    std::pair< bool, std::string> CommandInterface::script( const std::string& load_command )
    {
        std::string script_target ( load_command );
        ltrim(script_target);

        bool executed = false;

        for( std::list< boost::filesystem::path >::iterator it = paths.begin();
                it != paths.end();
                it++ )
        {

            std::string putative_target = (it->string() ) + script_target + ".L3";

            std::cout << putative_target << std::endl;

            std::ifstream script( putative_target.c_str() );

            if ( !script.good() )
                continue;

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
                // Blank line
                if( it->size() == 0 )
                    continue;


                std::pair< bool, std::string > result = (this->execute( *it ));
                if( !result.first )
                    return std::make_pair( false, "CI::Script <" + script_target + "> failed @ " + *it + "\n{ " + result.second + " }\n" );
          
            }
            
            executed = true;

        }

        if( executed )
            return std::make_pair( true, "CI::Script <" + script_target + ">.L3 succeeded" );
        else
            return std::make_pair( false, "CI::Script <" + script_target + ">.L3 does not exist" );

    }

    std::pair< bool, std::string> CommandInterface::printPath( const std::string& load_command )
    {
        std::stringstream ss;
        for( std::list< boost::filesystem::path >::iterator it = paths.begin();
                it != paths.end();
                it++ )
        {
          
            ss << it->string() << ":";

        }
            
        return std::make_pair( true, ss.str() );
    }

    std::pair< bool, std::string> CommandInterface::addPath( const std::string& load_command )
    {
      
        std::string load( load_command );

        ltrim( load );

        boost::filesystem::path putative_path( load );

        if( boost::filesystem::is_directory( putative_path ) )
        {
            paths.push_back( putative_path );
            return std::make_pair( true, "CI::Path <" + load_command+ "> added" );
        }
        else
            return std::make_pair( false, "CI::Path <" + load_command+ "> does not exist" );
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

            // Create pose sequence
            boost::shared_ptr< std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > > poses( new std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > () );
            
            // And extract them
            pose_reader->extract( *poses );

            // Here, we add a new component::leaf with the poses
            boost::shared_ptr< L3::Visualisers::PoseSequenceRenderer > sequence( new L3::Visualisers::PoseSequenceRenderer( poses ) );

            // Add it in as an extra
            layout->addExtra( dataset_target, sequence );

            // Keep track, in case we want to delete them
            added_trajectories.push_back( dataset_target );

        }
        catch( L3::no_such_folder& except )
        {
            return std::make_pair( false, "CI::Failed to add <" + dataset_target + ">" + "[" + except.what() + "]"  );
        }
            
        return std::make_pair( true, "CI::Added <" + dataset_target + ">" );

    }

    std::pair< bool, std::string> CommandInterface::removeTrajectory( const std::string& command )
    {
        if( !layout )
            return std::make_pair( false, "CI::No associated layout" );

        std::string trajectory_target( command );

        if( layout->removeExtra( trajectory_target ) )
            return std::make_pair( true, "CI::Removed <" + trajectory_target + ">" );
        else
            return std::make_pair( true, "CI::Failed to remove <" + trajectory_target + ">" );

    }

    std::pair< bool, std::string> CommandInterface::removeTrajectories( const std::string& command )
    {
        if( !layout )
            return std::make_pair( false, "CI::No associated layout" );
        else
        {

            for( std::list< std::string >::iterator it = added_trajectories.begin();
                    it != added_trajectories.end();
                    it++ )
                this->removeTrajectory( *it );

            return std::make_pair( true, "CI::Trajectories cleared" );
        }
    }

    std::pair< bool, std::string> CommandInterface::clear( const std::string& command )
    {
        if( !layout )
            return std::make_pair( false, "CI::No associated layout" );
        else
        {
            container->mission.reset();
            container->runner.reset();
            container->dataset.reset();
            return std::make_pair( true, "CI::Clear" );
        }
    }

    std::pair< bool, std::string> CommandInterface::history( const std::string& command )
    {
        return std::make_pair( true, "CI::Clear" );
    }

    std::pair< bool, std::string > CommandInterface::stop( const std::string& command )
    {
        container->runner->paused = true;
        return std::make_pair( true, "CI::Stopped" );
    }

    std::pair< bool, std::string > CommandInterface::start( const std::string& command )
    {
        container->runner->paused = false;
        return std::make_pair( true, "CI::Started" );
    }


    std::pair< bool, std::string > CommandInterface::runMode( const std::string& command )
    {
        container->runner->paused = true;

        std::string command_copy(command);

        ltrim(command_copy);

        if( command_copy == "step" )
            container->runner->run_mode = L3::RunMode::Step;
        else if(  command_copy == "cont" )
            container->runner->run_mode = L3::RunMode::Continuous;
        else
        {
            container->runner->paused = false;
            return std::make_pair( false, "CI::Unknown Mode <" + command_copy + ">"  );
        }

        return std::make_pair( true, "CI::Mode" );
    }

    std::string CommandInterface::getState()
    {
        return "";
    }
}
