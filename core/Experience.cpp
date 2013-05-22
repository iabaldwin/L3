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
 *  Selection Policies
 */

bool StrictlyRetrospectivePolicy::operator()( std::deque< experience_section>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window )
{
    /*
     *  Initialise distances
     */
    std::vector< std::pair< double, unsigned int > > distances;

    /*
     *  Calculate distances to all sections
     */
    for( unsigned int i=0; i<sections->size(); i++ )
        distances.push_back( std::make_pair( norm( std::make_pair( x, y ), std::make_pair( (*sections)[i].x, (*sections)[i].y)  ), i ) ); 
   
    /*
     *  Sort the distances
     */
    std::sort( distances.begin(), distances.end() );

    required_sections.clear();
    
    int window_cpy(window);

    //for( int i=0; i< distances.size();i++ )
        //std::cout << distances[i].second << " ";
    //std::cout << std::endl << "------------------------" << std::endl;

    //Here is the magic
    for( unsigned int i=distances.front().second; window_cpy-->0  && i>=0; i-- )
        required_sections.push_front( i );
    
    return true;

}

bool KNNPolicy::operator()( std::deque< experience_section>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window )
{
    /*
     *  Initialise distances
     */
    std::vector< std::pair< double, unsigned int > > distances;

    /*
     *  Calculate distances to all sections
     */
    for( unsigned int i=0; i<sections->size(); i++ )
        distances.push_back( std::make_pair( norm( std::make_pair( x, y ), std::make_pair( (*sections)[i].x, (*sections)[i].y)  ), i ) ); 
   
    /*
     *  Sort the distances
     */
    std::sort( distances.begin(), distances.end() );
    std::vector< std::pair< double, unsigned int > >::iterator distances_iterator = distances.begin();

    /*
     *  Build up a list of required sections
     */
    required_sections.clear();
    for( unsigned int i=0; i<window && i<distances.size(); i++ )
        required_sections.push_front( distances_iterator++->second );
    
    return ( !required_sections.empty() );
}


/*
 *  Experience
 */

Experience::Experience( std::deque<experience_section> sections, 
            std::string& fname, 
            boost::shared_ptr< SelectionPolicy > policy,
            int window ) 
                : sections(sections), 
                    window(window),
                    running(true),
                    _x(0.0), _y(0.0),
                    policy(policy)
{
    // Open 
    data.open( fname.c_str(), std::ios::binary );

    // Allocate
    std::vector<double> densities;
    densities.push_back( .5 );
    densities.push_back( 1 );
    densities.push_back( 2.5 );

    experience_pyramid.reset( new L3::HistogramPyramid<double>( densities ) );
    resident_point_cloud.reset( new L3::PointCloud<double>() );

    // Go
    thread.start( *this );

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
        /*
         *  Choose the sections required according to some policy
         */
        std::list<unsigned int> required_sections;
        if ( !policy->operator()( &sections, _x, _y, required_sections, window ) )  // Catastrophe
            break;

        /*
         *  Mark everything as *NOT* required
         */
        for( std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloud<double> > > >::iterator map_it = resident_sections.begin();
                map_it != resident_sections.end();
                map_it++ )
            map_it->second.first = false;

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
                
                //Load 
                std::pair< unsigned int, L3::Point<double>* > load_result = load(*it);

                L3::PointCloud<double>* cloud = new L3::PointCloud<double>();

                //L3::allocate( cloud, load_result.first );
                //std::copy( load_result.second, load_result.second + load_result.first, cloud->points );
                //delete [] load_result.second;

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
        std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloud<double> > > >::iterator map_it = resident_sections.begin();
        while( map_it != resident_sections.end() )
        {
            if( !map_it->second.first )
            {
                update_required = true; 
                map_it->second.second.reset(); 
                resident_sections.erase( map_it++ );
            }
            else
                clouds.push_back( map_it++->second.second ); 
            
        }

        /*
         *Assign the resident cloud
         */
        if (update_required)
        {
            // Join clouds
            join( clouds, resident_point_cloud );

            if ( resident_point_cloud->num_points != 0 )
            {
                // Compute histogram
                //std::pair<double,double> min_bound = L3::min<double>( &*resident_point_cloud );
                //std::pair<double,double> max_bound = L3::max<double>( &*resident_point_cloud );
                //std::pair<double,double> means     = L3::mean( &*resident_point_cloud );

                boost::tuple<double,double,double> min_bound = L3::min<double>( &*resident_point_cloud );
                boost::tuple<double,double,double> max_bound = L3::max<double>( &*resident_point_cloud );
                boost::tuple<double,double,double> means     = L3::mean( &*resident_point_cloud );



                //L3::BoxSmoother< double, 3 > smoother; 
                L3::GaussianSmoother< double > smoother; 

                for( L3::HistogramPyramid<double>::PYRAMID_ITERATOR it = this->experience_pyramid->begin();
                        it != this->experience_pyramid->end();
                        it++ )
                {

                    boost::shared_ptr<L3::HistogramUniformDistance<double> > current_histogram = boost::dynamic_pointer_cast<L3::HistogramUniformDistance<double> >(*it);

                    WriteLock lock( current_histogram->mutex );

                    //current_histogram->create(  means.first, 
                            //min_bound.first, 
                            //max_bound.first,
                            //means.second,                   
                            //min_bound.second, 
                            //max_bound.second );

                    current_histogram->create(  means.get<0>(), 
                            min_bound.get<0>(), 
                            max_bound.get<0>(),
                            means.get<1>(),                   
                            min_bound.get<1>(), 
                            max_bound.get<1>());



                    current_histogram->operator()( resident_point_cloud.get() );
                    smoother.smooth( current_histogram.get() );
                    lock.unlock();
                
                }
            }
        }

        // Play nice
        usleep( .2*1e6 );
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
