#include <iostream>
#include "Reader.h"
#include "Dataset.h"
#include "Utils.h"
#include "PointCloud.h"
#include "Projector.h"
#include "DataAdapters.h"

#include <sstream>
#include <libconfig.h++>
#include <string.h>

int main()
{

    FILE* f = fopen( "/Users/ian/code/matlab/conf/dataset/2012-02-08-09-36-42-WOODSTOCK-SLOW.config", "r"  );

    libconfig::Config config;
    config.read( f );

    for ( int i=0; i<10; i++)
    {
        std::stringstream ss;
        ss <<  "mission.lasers.[" << i << "]";

        std::string current_name;
        if (config.lookupValue( ss.str()+".name", current_name) )
            std::cout << current_name << std::endl;
    
        if ( strcmp( current_name.c_str() , "LMS1xx_10420001" ) == 0 )
        {
            double res;
            if ( config.lookupValue( ss.str()+".frequency", res ) )
                std::cout << res << std::endl;
        }
    }

}
