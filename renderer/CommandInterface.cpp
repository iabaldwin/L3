#include "CommandInterface.h"

static inline std::string& ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}


namespace L3
{
    //CommandInterface::CommandInterface( L3::Visualisers::EstimatorLayout* layout, boost::shared_ptr< L3::Container > container )
    CommandInterface::CommandInterface( L3::Visualisers::DatasetLayout* layout, boost::shared_ptr< L3::Container > container )
        : layout(layout), container(container)
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
                std::string dataset_target = command.substr( pos+4, command.size() );

                ltrim(dataset_target);

                //try
                //{
                    //std::cout << container->dataset.use_count() << std::endl;
                    //container->dataset.reset();
                    //std::cout << container->dataset.use_count() << std::endl;
           
                    //container->dataset.reset( new L3::Dataset( dataset_target ) );
                    //container->dataset.reset();

                    //if( !(container->dataset->validate() &&container->dataset->load() ) )
                        //return std::make_pair( false, "L3::Could not validate: " + dataset_target ); 
                                    
                    return std::make_pair( true, "L3::Reset done: " + dataset_target ); 
                //}
                //catch( ... )
                //{
                    //return std::make_pair( false, "L3::No such directory: " + dataset_target ); 
                //}

                try
                {
                    //container->mission.reset( new L3::Configuration::Mission( *container->dataset ) );
                }
                catch( ... )
                {
                    return std::make_pair( false, "L3::No such configuration: " + dataset_target ); 
                }

                try
                {
                    // Pose oracle
                    //container->oracle_source.reset( new L3::ConstantTimeIterator< L3::SE3 >( container->dataset->pose_reader ) );
                    //// Pose iterator
                    //container->integrated_pose_iterator.reset( new L3::ConstantTimeIterator< L3::LHLV >( container->dataset->LHLV_reader ) );

                    //// LIDARS
                    //container->vertical_LIDAR_iterator.reset( new L3::ConstantTimeIterator<L3::LMS151>(  container->dataset->LIDAR_readers[ container->mission->declined ] ) );
                    //container->horizontal_LIDAR_iterator.reset( new L3::ConstantTimeIterator<L3::LMS151>(  container->dataset->LIDAR_readers[ container->mission->horizontal ] ) );

                    //// Windowers
                    //container->oracle.reset( new L3::ConstantTimeWindower<L3::SE3>( container->oracle_source.get() ) );
                    //container->pose_windower.reset( new L3::ConstantTimeWindower< L3::LHLV> ( container->integrated_pose_iterator.get() ) );
                
                    //// Swathe builder
                    //container->swathe_builder.reset( new L3::SwatheBuilder( container->pose_windower.get(), container->vertical_LIDAR_iterator.get() ) );

                    //// Cloud
                    //container->point_cloud.reset( new L3::PointCloud<double>()) ;

                    //L3::SE3 projection = L3::SE3::ZERO();
                    //L3::Configuration::convert( container->mission->lidars[ container->mission->declined], projection );
 
                    //container->projector.reset( new L3::Projector<double>( &projection, container->point_cloud.get() ) );
                }
                catch( ... )
                {
                    return std::make_pair( false, "L3::Unable to build core: " + dataset_target ); 
                }

                return std::make_pair( true, "L3::Loaded dataset \t\t<" + dataset_target + ">" );
            }
        }


        {
            size_t pos = command.find( "exp" );

            if ( pos != std::string::npos )
            {
                std::string dataset_target = command.substr( pos+3, command.size() );

                ltrim(dataset_target);

                try
                {
                    L3::Dataset experience_dataset( dataset_target );
                    L3::ExperienceLoader experience_loader( experience_dataset );
                    container->experience = experience_loader.experience;
                }
                catch( ... )
                {
                    return std::make_pair( false, "L3::No such experience: " + dataset_target ); 
                }

                return std::make_pair( true, "L3::Loaded experience \t\t<" + dataset_target + ">" );
            }
        }

        {
            size_t pos = command.find( "algo" );

            if ( pos != std::string::npos )
            {
                if( !container->experience )
                    return std::make_pair( retval, "L3::No current experience" );

                try
                {
                    //container->cost_function.reset( new L3::Estimator::KLCostFunction<double>() );
                    //container->algorithm.reset( new L3::Estimator::IterativeDescent<double>( container->cost_function.get(), container->experience->experience_pyramid ) );
                }
                catch( ... )
                {
                    return std::make_pair( false, "L3::Unable to build estimation core " );
                }

                return std::make_pair( true, "L3::Loaded< Algorithm >" );

            }

        }

        /*
         *Runner
         */
        {

            size_t pos = command.find( "run" );

            if ( pos != std::string::npos )
            {
                if (!container)
                    return std::make_pair( retval, "L3::No associated container" );

                if( !container->experience )
                    return std::make_pair( retval, "L3::No current experience" );
         
                return std::make_pair( true, "L3::Running" );

            }

        }

        /*
         *  Scripting
         */
        {
            {
                size_t pos = command.find( "script" );

                if ( pos != std::string::npos )
                {
                    std::string script_target = command.substr( pos+6, command.size() );

                    ltrim(script_target);

                    std::ifstream script( script_target.c_str() );

                    std::list< std::string > script_history;

                    std::string line;

                    while( getline( script, line ) )
                        script_history.push_back( line );

                    script.close();

                    for( std::list< std::string  >::iterator it = script_history.begin();
                            it != script_history.end(); 
                            it++ )
                    {
                        if( !(this->execute( *it )).first )
                            return std::make_pair( false, "L3::Script <" + script_target + "> failed @ " + *it );

                    }

                    return std::make_pair( true, "L3::Script <" + script_target + "> succeeded" );
                }

            }

        }

        /*
         *  Stop
         */
        {
            size_t pos = command.find( "stop" );

            if ( pos != std::string::npos )
            {
                //std::cout << "Stop called" << std::endl;
                //container->runner.stop();
            
                return std::make_pair( true, "L3::Stopped" );
            }
        
        }

        /*
         *  Reset
         */
        {
            size_t pos = command.find( "reset" );

            if ( pos != std::string::npos )
            {
                return std::make_pair( true, "L3::Reset" );
            }
        }


        /*
         *  Quit
         */
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
