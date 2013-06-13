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
                    //swathe.push_back( std::make_pair( index->second, it->second ) );
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
      
        std::cout.precision( 15 );

        // Rebuild swathe completely
        swathe.clear();

        // Find the new data, between the last update time and now
        L3::Iterator<L3::LMS151>::WINDOW_ITERATOR index = std::lower_bound( LIDAR_iterator->window.begin(), 
                LIDAR_iterator->window.end(), 
                previous_update,
                LIDAR_comparator );

        _window_buffer.insert( _window_buffer.end(), index, LIDAR_iterator->window.end() );

        L3::Iterator<L3::SE3>::WINDOW_ITERATOR it = pose_windower->window->begin() ;
   
        // For each lidar pose, find the nearest scan
        while( it != pose_windower->window->end() )
        {
            // Nearest time
            L3::Iterator<L3::LMS151>::WINDOW_ITERATOR index = std::lower_bound( _window_buffer.begin(), 
                    _window_buffer.end(),
                    it->first, 
                    LIDAR_comparator );

            if ( index == _window_buffer.end() )
            {
                std::cerr << "LIDAR" << std::endl;
                for(std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > >::iterator lidar_it = LIDAR_iterator->window.begin();
                        lidar_it != LIDAR_iterator->window.end();
                        lidar_it++)
                    std::cerr << lidar_it->first << std::endl;

                std::cerr << "-------------" << std::endl;
                std::cerr << "TMP BUFFER" << std::endl;
                
                for(std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > >::iterator buf_it = _window_buffer.begin();
                        buf_it != _window_buffer.end();
                        buf_it++)
                    std::cerr << buf_it->first << std::endl;
                std::cerr << "-------------" << std::endl;
                std::cerr << it->first << std::endl;

                std::cerr << previous_update << std::endl;

                std::cerr << "Missing data." << std::endl;
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
