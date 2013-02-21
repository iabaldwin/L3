#include "Datatypes.h"
#include "WindowerFactory.h"

int main()
{

    std::string dataset_binary( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/LMS1xx_10420001_192.168.0.51.lidar" );
    L3::WindowerFactory<L3::Pose>::constantTimeWindow( dataset_binary, 10.0 );

    std::string dataset_text( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
    L3::WindowerFactory<L3::Pose>::constantTimeWindow( dataset_text, 10.0 );
}

