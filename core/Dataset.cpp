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

std::ostream& operator<<( std::ostream& o, const Dataset& dataset )
{
    std::copy(boost::filesystem::directory_iterator( dataset.root_path ), 
            boost::filesystem::directory_iterator(), 
            std::ostream_iterator<boost::filesystem::directory_entry>(o, "\n")); 

    return o;
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
   
    if( !(pose_reader->read() && pose_reader->extract( poses ) ) )
        throw std::exception();

    // Load LIDARs
    std::list< boost::filesystem::directory_entry >::iterator it = LIDARs.begin();

    std::auto_ptr<L3::IO::LIDARReader> scan_reader( new L3::IO::LIDARReader() );
    
    while ( it != LIDARs.end() )
    {
        scan_reader->open( (*it).path().string() );
        scan_reader->read();
        
        std::vector<L3::LMS151*> raw_scans;

        if ( !scan_reader->extract( raw_scans ) )
            throw std::exception();
        else
        {
            std::string raw_name = (*it).path().leaf().string();
            std::string LIDAR_name( raw_name.begin(), raw_name.begin()+15); 
            LIDAR_names.push_back( LIDAR_name );
            LIDAR_data[LIDAR_name] = raw_scans; 
        }
        
        it++;
    }

}

template <typename T>
struct Comparator
{
    bool operator()( T* t, const double f )
    //bool operator()( const double f, T* t  )
    {
        return (f > t->time);
        //return (f < t->time);
    }
};



L3::Pose* Dataset::getPoseAtTime( double time )
{
    assert( poses.size() > 0 );
    std::vector<L3::Pose*>::iterator it;

    Comparator<L3::Pose> c;

    it = std::lower_bound( poses.begin(), poses.end(), time, c );
    //it = std::upper_bound( poses.begin(), poses.end(), time, c );

    L3::Pose* p = 0;

    if( it != poses.end() )
    {
        p = &*(*it);     
    }

    return p;
}

L3::LMS151* Dataset::getScanAtTime( double time, const std::string& name )
{
    //assert( poses.size() > 0 );
    //std::vector<L3::LMS151*>::iterator it;

    L3::LMS151* l = 0;

    //Comparator<L3::LMS151> c;

    //it = std::lower_bound( LIDAR_data[name].begin(), LIDAR_data[name].end(), time, c );

    //if( it != LIDAR_data[name].end() )
    //{
        //l = &*(*it);     
    //}

    return l;
}


}
