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

}

}