#include "Dataset.h"

namespace L3
{

enum extensionType { INS, LIDAR, LHLV };


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
    lookup[".lhlv"] = LHLV;
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
     *  2.  1 X LHLV file
     *  3.  N x LIDAR files 
     *
     */

    boost::filesystem::directory_iterator it( root_path );

    while( it != boost::filesystem::directory_iterator() )
    {
        switch ( lookup[boost::filesystem::extension( *it )])
        {
            case INS:
                OxTS_ins = *it;
                break;
            
            case LIDAR:
                LIDARs.push_front( *it );
                break;
       
            case LHLV:
                OxTS_lhlv = *it;
                break;

            default:
                break;
        
        }
        
        it++;
    }

    // Validate logic
    if ( !( boost::filesystem::exists( OxTS_ins )) || 
            !( boost::filesystem::exists( OxTS_lhlv) ) || 
                (LIDARs.size()) == 0 ) 
            return false;

    return true;
}

bool Dataset::load()
{
    L3::Tools::Timer t;
    t.begin();

    // Load INS
    std::auto_ptr<L3::IO::PoseReader> pose_reader( new L3::IO::PoseReader() );
    pose_reader->open( OxTS_ins.path().string() );
  
    if( !(pose_reader->read() && pose_reader->extract( poses ) ) )
        throw std::exception();

    std::auto_ptr<L3::IO::LHLVReader> lhlv_reader( new L3::IO::LHLVReader() );
    lhlv_reader->open( OxTS_lhlv.path().string() );
  
    if( !(lhlv_reader->read()  ) )
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
          
            /*
             *Logging always used to be:
             *  -> LMS_XXX_XXXX
             *  but then at some stage, the 
             *  logger started reporting
             *  IP addresses as well, so 
             *  we have to cull the string
             *  so that we obtain only 
             *  the LIDAR name and serial.
             */
            std::string LIDAR_name( raw_name.begin(), raw_name.begin()+15); 
            LIDAR_names.push_back( LIDAR_name );
            LIDAR_data[LIDAR_name] = raw_scans; 
        }
        
        it++;
    }

    std::cout << "Loaded: " << t.end() << "s" << std::endl;
}

template <typename T>
struct Comparator
{
    bool operator()( T* t, const double f )
    {
        return (t->time < f);
    }
};

template <typename T>
struct Sorter
{
    bool operator()( T* t1,  T* t2 )
    {
        return ( t1->time > t2->time );
    }
};


L3::Pose* Dataset::getPoseAtTime( double time )
{
    assert( poses.size() > 0 );
    std::vector<L3::Pose*>::iterator it;

    Comparator<L3::Pose> c;

    it = std::lower_bound( poses.begin(), poses.end(), time, c );

    L3::Pose* p = 0;

    if( it != poses.end() )
    {
        p = &*(*it);     
    }

    return p;
}

L3::LMS151* Dataset::getScanAtTime( double time, const std::string& name )
{
    assert( poses.size() > 0 );
    std::vector<L3::LMS151*>::iterator it;

    L3::LMS151* l = 0;

    Comparator<L3::LMS151> c;

    it = std::lower_bound( LIDAR_data[name].begin(), LIDAR_data[name].end(), time, c );

    if( it != LIDAR_data[name].end() )
    {
        l = &*(*it);     
    }

    return l;
}


}
