#include "Experience.h"

namespace L3
{

template <typename T>
T norm( std::pair<T,T> a, std::pair<T,T> b)
{
    return sqrt( pow( a.first-b.first,2) + pow( a.second - b.second, 2) );
}


std::ostream& operator<<( std::ostream& o, experience_section section )
{
    o << section.id << ":" << section.x << "," << section.y << "(" << section.stream_position << "," << section.payload_size << ")";

    return o;
}

bool operator<( std::pair< double, unsigned int > a, std::pair< double, unsigned int > b )
{
    return a.first < b.first;
}

/*
 *  Experience
 */

Experience::Experience( std::deque<experience_section>  SECTIONS, 
            std::string& fname, 
            unsigned int WINDOW
            ) : sections(SECTIONS), 
                window(WINDOW),
                running(true)
{
    // Open 
    data.open( fname.c_str(), std::ios::binary );

    // Go
    thread.start( *this );

    experience_histogram.reset( new L3::HistogramUniformDistance<double>() ); 
    
    resident_point_cloud.reset( new L3::PointCloud<double>() );
}

Experience::~Experience()
{
    data.close();          
    running = false;        
    if( thread.isRunning() )
        thread.join();          
}

void Experience::run()
{
    while( running )
    {
        std::vector< std::pair< double, unsigned int > > distances;

        // Calculate distances to all sections
        for( unsigned int i=0; i<sections.size(); i++ )
        {
            distances.push_back( std::make_pair( norm( std::make_pair( _x, _y ), std::make_pair( sections[i].x, sections[i].y)  ), i ) ); 
        }

        // Sort the distances
        std::sort( distances.begin(), distances.end() );
        std::vector< std::pair< double, unsigned int > >::iterator distances_iterator = distances.begin();

        /*
         *  Build up a list of required sections
         */
        std::list<unsigned int> required_sections;
        for( unsigned int i=0; i<window-1 && i<distances.size(); i++ )
        {
            required_sections.push_front( distances_iterator++->second );
        }

        // Mark everything as *NOT* required
        for( std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloud<double> > > >::iterator map_it = resident_sections.begin();
                map_it != resident_sections.end();
                map_it++ )
        {
            map_it->second.first = false;
        }

        /*
         *  Search
         */
        std::list< boost::shared_ptr<L3::PointCloud<double> > > clouds;

        bool update_required = false;

        for ( std::list<unsigned int>::iterator it = required_sections.begin(); it != required_sections.end(); it++ )
        {
            std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloud<double> > > >::iterator map_it = resident_sections.find( *it );

            //We need it, and don't have it
            if ( map_it == resident_sections.end() )
            {
                update_required = true; 
                //Load the pair
                std::pair< unsigned int, L3::Point<double>* > load_result = load(*it);

                L3::PointCloud<double>* cloud = new L3::PointCloud<double>();

                cloud->num_points = load_result.first;
                cloud->points = load_result.second;

                // Insert, mark it as required by default
                resident_sections.insert( std::make_pair( *it, std::make_pair( true, boost::shared_ptr<L3::PointCloud<double> >( cloud ) ) ) );

            }
            else
            {
                // We need it, and we have it
                map_it->second.first = true;
            }
        }

        /*
         *Erase everything *NOT* required
         */
        for( std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloud<double> > > >::iterator map_it = resident_sections.begin();
                map_it != resident_sections.end();
                map_it++ )
        {
            if( !map_it->second.first )
            {
                update_required = true; 
                resident_sections.erase( map_it );
            }
            else
                clouds.push_back( map_it->second.second ); 
            
        }


        // Assign the resident cloud
        if (update_required)
        {
            WriteLock writer( experience_histogram->mutex );
            
            // Recompute histogram
            join( clouds, resident_point_cloud );

            // Compute histogram
            std::pair<double,double> min_bound = L3::min<double>( &*resident_point_cloud );
            std::pair<double,double> max_bound = L3::max<double>( &*resident_point_cloud );
            std::pair<double,double> means     = L3::mean( &*resident_point_cloud );

            //boost::dynamic_pointer_cast<L3::HistogramUniformDistance<double> >(experience_histogram)->create( means.first, 
            boost::dynamic_pointer_cast<L3::HistogramUniformDistance<double> >(experience_histogram)->create( means.first, 
                    min_bound.first, 
                    max_bound.first,
                    means.second, 
                    min_bound.second, 
                    max_bound.second );

            (*experience_histogram)( &*resident_point_cloud );
        
        }

        // Play nice
        usleep( .1*1e6 );
    }
}

bool Experience::update( double x, double y )
{
    _x = x;
    _y = y;
    return true;
}

std::pair< long unsigned int, L3::Point<double>* > Experience::load( unsigned int id )
{
    if( id >= sections.size() )
        throw std::exception();

    // Seek
    data.seekg( sections[id].stream_position, std::ios_base::beg );
    char* tmp = new char[sections[id].payload_size];

    // Read 
    data.read( tmp, sections[id].payload_size );

    // DBG checks
    assert( data.good() ); 
    assert( sections[id].payload_size == data.gcount() );

    return std::make_pair( sections[id].payload_size/sizeof(L3::Point<double>), reinterpret_cast<L3::Point<double>*>( tmp ) );
}

}
