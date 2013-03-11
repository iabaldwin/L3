#include "L3.h"

int main()
{
    std::auto_ptr<L3::IO::BinaryReader< L3::SE3 > > reader( new L3::IO::BinaryReader<L3::SE3>() );
    
    reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
    reader->read();

    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses;
    
    LengthEstimatorInterface length_estimator;

    double accumulate = 0.0;

    if ( reader->extract( poses ) )
    {
        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it; 
        
        for ( it = poses.begin(); it != poses.end(); it++ )
        {
            std::cout << (accumulate += length_estimator( *it ) ) << std::endl;
        }

    }
}
