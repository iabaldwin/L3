#include "SwatheBuilder.h"

namespace L3
{

    typedef std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LMS151> >, std::pair<double, boost::shared_ptr<L3::LMS151> >&, std::pair<double, boost::shared_ptr<L3::LMS151> >*> PTR;

    bool RawSwatheBuilder::update( double )
    {
        //Rebuild swathe completely
        swathe.clear();
      
        if( LIDAR_iterator->window.size() < pose_windower->window->size() )
        {
            L3::Iterator<L3::LMS151>::WINDOW_ITERATOR it = LIDAR_iterator->window.begin() ;

            //For each lidar scan, find the nearest pose
            while( it != LIDAR_iterator->window.end() )
            {
                // Nearest time
                L3::Iterator<L3::SE3>::WINDOW_ITERATOR index = std::lower_bound( pose_windower->window->begin(), 
                        pose_windower->window->end(), 
                        it->first, 
                        pose_comparator );

                if ( index == pose_windower->window->end() ) 
                    break;

                if ((index->first - it->first) == 0)
                    swathe.push_back( std::make_pair( index->second, it->second ) );

                it++;
            }
        }
        else
        {
            L3::Iterator<L3::SE3>::WINDOW_ITERATOR it = pose_windower->window->begin() ;
            
            // For each lidar pose, find the nearest scan
            while( it != pose_windower->window->end() )
            {
                // Nearest time
                L3::Iterator<L3::LMS151>::WINDOW_ITERATOR index = std::lower_bound( LIDAR_iterator->window.begin(), 
                        LIDAR_iterator->window.end(), 
                        it->first, 
                        LIDAR_comparator );

                if ((index->first - it->first) == 0)
                    swathe.push_back( std::make_pair( it->second, index->second ) );

                it++;
            }

        }

        return true;

    }

    bool BufferedSwatheBuilder::update( double )
    {
        if( LIDAR_iterator->window.empty() || pose_windower->window->empty() )
            return false;

        // Rebuild swathe completely
        swathe.clear();

#ifndef NDEBUG
        std::cout << LIDAR_iterator->window.size() << ":" << pose_windower->window->size() << std::endl;
#endif

        // Find the new data, between the last update time and now
        L3::Iterator<L3::LMS151>::WINDOW_ITERATOR index = std::lower_bound( LIDAR_iterator->window.begin(), 
                LIDAR_iterator->window.end(), 
                previous_update,
                LIDAR_comparator );

        if( index->first == previous_update )
            index++;

        _window_buffer.insert( _window_buffer.end(), index, LIDAR_iterator->window.end() );

#ifndef NDEBUG
        for( std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > >::iterator it = (_window_buffer.begin()+1);
                it != _window_buffer.end(); 
                it++ )
        {
            double previous = (it-1)->first;
            double current = it->first;

            if( (current-previous) > .2 ) 
            {
                std::cout << "BIG gap" << std::endl;
                exit(-1);
            }
            if ((current-previous) < .001 )
            {
                std::cout << current-previous << std::endl;
                std::cout << "SMALL gap" << std::endl;
                exit(-1);
            }
        }
#endif
        L3::Iterator<L3::SE3>::WINDOW_ITERATOR it = pose_windower->window->begin();
   
        // For each lidar pose, find the nearest scan
        while( it != pose_windower->window->end() )
        {
            // Nearest time
            L3::Iterator<L3::LMS151>::WINDOW_ITERATOR index = 
                std::lower_bound( _window_buffer.begin(), 
                    _window_buffer.end(),
                    it->first, 
                    LIDAR_comparator );

            if ( index == _window_buffer.end() )
            {
                std::cout << "(" << __FILE__ << ")" << " Lookup failure" << std::endl;

                std::cerr.precision( 15 );
                
                std::cerr << "LIDAR (" << LIDAR_iterator->window.size() << ")" << std::endl;
                for(std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > >::iterator lidar_it = LIDAR_iterator->window.begin();
                        lidar_it != LIDAR_iterator->window.end();
                        lidar_it++)
                    std::cerr << lidar_it->first << std::endl;
                std::cerr << "-------------" << std::endl;
                
                std::cerr << "BUFFER (" <<  _window_buffer.size() << ")" << std::endl;
                for(std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > >::iterator buf_it = _window_buffer.begin();
                        buf_it != _window_buffer.end();
                        buf_it++)
                    std::cerr << buf_it->first << std::endl;
                std::cerr << "-------------" << std::endl;

                std::cerr << "Pose window(" <<  pose_windower->window->size() << ")" << std::endl;
                for(std::deque< std::pair< double, boost::shared_ptr< L3::SE3 > > >::iterator _iterator = pose_windower->window->begin();
                        _iterator != pose_windower->window->end();
                        _iterator++)
                    std::cerr << _iterator->first << std::endl;
                std::cerr << "-------------" << std::endl;

                std::cerr << "Query time: "  << it->first << std::endl;
                std::cerr << "Previous time: " << previous_update << std::endl;
                
                exit(-1);
            }

            if (( index->first - it->first) == 0 )
                swathe.push_back( std::make_pair( it->second, index->second ) );
            
            it++;
        }

        previous_update = _window_buffer.back().first;

        index = std::lower_bound( _window_buffer.begin(),
                _window_buffer.end(), 
                pose_windower->window->front().first,
                LIDAR_comparator );

        _window_buffer.erase( _window_buffer.begin(), index );

        return true;
    }

}
