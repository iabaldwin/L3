#include "Definitions.h"
#include "Reader.h"
#include "Utils.h"

int main()
{

    std::auto_ptr<L3::IO::BulkReader<L3::Pose> > reader( new L3::IO::BulkReader<L3::Pose>() );
    
    reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
    reader->read();

    std::vector< std::pair< double, boost::shared_ptr<L3::Pose> > > poses;
    if ( !reader->extract( poses ) )
        throw std::exception();

    L3::Utils::localisePoseChainToMean( poses.begin(), poses.end() );
    
    std::vector< std::pair< double, boost::shared_ptr<L3::Pose> > > new_poses( poses.size() );

    L3::Utils::localisePoseChainToMean( poses.begin(), poses.end(), new_poses.begin() );

}
