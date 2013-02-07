#include "Dataset.h"


namespace L3
{

Dataset::Dataset() 
{

}

Dataset::Dataset( const std::string& target ) 
{
    root_path /= target;
    root_path /= "L3";

    // Does the directory exist?
    if ( !boost::filesystem::is_directory( root_path ))
        throw std::exception();

    lookup[".ins"] = INS;
    lookup[".lidar"] = LIDAR;

}

Dataset& Dataset::list()
{
    //std::copy(boost::filesystem::directory_iterator( root_path ), 
            //boost::filesystem::directory_iterator(), 
            //std::ostream_iterator<boost::filesystem::directory_entry>(std::cout, "\n")); 

    return *this;
}

bool Dataset::validate()
{

    /*
     *For any dataset, there should be:
     *  1.  1 x INS file
     *  2.  N x LIDAR files 
     *
     *
     */

    boost::filesystem::directory_iterator it( root_path );

    while( it != boost::filesystem::directory_iterator() )
    {
        switch ( lookup[boost::filesystem::extension( *it )])
        {
            case INS:
                OxTS = *it;
                break;
            
            case LIDAR:
                LIDARs.push_front( *it );
                break;
            
            default:
                break;
        
        }
        
        it++;
    }

    // Validate logic
    return true;
}

bool Dataset::load()
{

    // Load INS
    std::auto_ptr<L3::IO::PoseReader> pose_reader( new L3::IO::PoseReader() );
    pose_reader->open( OxTS.path().string() );
    
    if( !pose_reader->read() )
        throw std::exception();

    std::vector<L3::Pose*> poses;
    if ( pose_reader->extract( poses ) )
        for( std::vector<L3::Pose*>::iterator it=poses.begin(); it!= poses.end(); it++ )
            std::cout << *(*it) << std::endl;

    // Load LIDARs
    std::list< boost::filesystem::directory_entry >::iterator it = LIDARs.begin();

    std::auto_ptr<L3::IO::LIDARReader> scan_reader( new L3::IO::LIDARReader() );
    
    while ( it != LIDARs.end() )
    {
        scan_reader->open( (*it).path().string() );

        std::vector<L3::LMS151*> scans;
        scan_reader->read();

        if ( scan_reader->extract( scans ) )
            for( std::vector<L3::LMS151*>::iterator it=scans.begin(); it!= scans.end(); it++ )
                std::cout << *(*it) << std::endl;

        it++;
    }

}


}
