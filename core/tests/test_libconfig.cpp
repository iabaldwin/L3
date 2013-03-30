#include "L3.h"

#include <sstream>
#include <libconfig.h++>
#include <sstream>

#include "Misc.h"

int main()
{
    std::list <std::string> datasets = L3::Misc::getDatasetConfigurations();

    for( std::list<std::string>::iterator it = datasets.begin(); it!= datasets.end(); it++ )
    {
        FILE* f = fopen( it->c_str(), "r" );
        assert(f);

        std::stringstream ss;
        for( std::string::iterator str_it = it->begin(); str_it != it->end(); str_it++ )
            ss << "-";

        std::cout << ss.str() << std::endl << it->c_str() << std::endl;


        libconfig::Config config;
        config.read( f );

        for ( int i=0; i<10; i++)
        {
            std::stringstream ss;
            ss <<  "mission.lasers.[" << i << "]";

            std::string current_name;
            if (config.lookupValue( ss.str()+".name", current_name) )
                std::cout << "\t" << current_name << std::endl;

            if ( strcmp( current_name.c_str() , "LMS1xx_10420001" ) == 0 )
            {
                double res;
                if ( config.lookupValue( ss.str()+".frequency", res ) )
                    std::cout << "\t\t" << res << std::endl;
            }

        }

        fclose(f);
    }
}
