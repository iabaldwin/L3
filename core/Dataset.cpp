#include "Dataset.h"

namespace L3
{

Dataset::Dataset( const std::string& target )  : start_time(0)
{
    root_path /= target;
    root_path /= "L3";

    // Does the directory exist?
    if ( !boost::filesystem::is_directory( root_path ))
        throw std::exception();

    lookup[".ins"] = INS_file;
    lookup[".lhlv"] = LHLV_file;
    lookup[".lidar"] = LIDAR_file;
}

Dataset::~Dataset()
{
    if (pose_reader)
        pose_reader->stop();
   
    if ( LHLV_reader )
        LHLV_reader->stop();

    for ( std::list< boost::shared_ptr< SlidingWindow<L3::LMS151> > >::iterator it = LIDAR_readers.begin(); it != LIDAR_readers.end(); it++ )
    {
        (*it)->stop();
    }

    for( std::list< Poco::Thread* >::iterator it= threads.begin(); it != threads.end(); it++ )
    {
        (*it)->join();
        delete *it;
    }
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
            case INS_file:
                OxTS_ins = *it;
                break;
            
            case LIDAR_file:
                LIDARs.push_front( *it );
                break;
       
            case LHLV_file:
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
    pose_reader = L3::WindowerFactory<L3::SE3>::constantTimeWindow( OxTS_ins.path().string(), 30 ) ;
    pose_reader->initialise(); 
    runnables.push_back( pose_reader );

    LHLV_reader = L3::WindowerFactory<L3::LHLV>::constantTimeWindow( OxTS_lhlv.path().string(), 30 ) ;
    LHLV_reader->initialise(); 
    runnables.push_back( LHLV_reader );

    // Load LIDARs
    std::list< boost::filesystem::directory_entry >::iterator it = LIDARs.begin();

    while ( it != LIDARs.end() )
    {
        boost::shared_ptr< SlidingWindow<L3::LMS151> > reader = L3::WindowerFactory<L3::LMS151>::constantTimeWindow( (*it).path().string(), 30 );
        reader->initialise(); 
        LIDAR_readers.push_back( reader );
        runnables.push_back( reader );
        
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
        std::string raw_name = (*it).path().leaf().string();
        std::string LIDAR_name( raw_name.begin(), raw_name.begin()+15); 
        LIDAR_names.push_back( LIDAR_name );
        
        it++;
    }

    for( std::list< boost::shared_ptr<Poco::Runnable> >::iterator it = runnables.begin(); it != runnables.end(); it++ )
    {
        Poco::Thread* thread = new Poco::Thread();
        thread->start( *(*it) );
        threads.push_back( thread );
    }

    // Get the dataset time - by definition, this is the time of the first pose
    start_time = pose_reader->window.begin()->first;

    return true;
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


//L3::Pose* Dataset::getPoseAtTime( double time )
//{
    //assert( poses.size() > 0 );
    //std::vector<L3::Pose*>::iterator it;

    //Comparator<L3::Pose> c;

    //it = std::lower_bound( poses.begin(), poses.end(), time, c );

    //L3::Pose* p = 0;

    //if( it != poses.end() )
    //{
        //p = &*(*it);     
    //}

    //return p;
//}

//L3::LMS151* Dataset::getScanAtTime( double time, const std::string& name )
//{
    //assert( poses.size() > 0 );
    //std::vector<L3::LMS151*>::iterator it;

    //L3::LMS151* l = 0;

    //Comparator<L3::LMS151> c;

    //it = std::lower_bound( LIDAR_data[name].begin(), LIDAR_data[name].end(), time, c );

    //if( it != LIDAR_data[name].end() )
    //{
        //l = &*(*it);     
    //}

    //return l;
//}


}
