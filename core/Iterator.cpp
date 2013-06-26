#include "Iterator.h"
#include "Misc.h"

namespace L3
{

//template <typename T>
//void Iterator<T>::getWindow( typename std::deque< std::pair< double, boost::shared_ptr<T> > >& _window)  
//{
    //L3::ReadLock lock( this->mutex );

    //_window.resize( this->window.size() );
    //std::copy( this->window.begin(), this->window.end(), _window.begin() );
   
    //lock.unlock();
//}

template <typename T>
bool ConstantTimeIterator<T>::update( double time )
{
    // Obtain the pointer to the windower
    boost::shared_ptr< L3::SlidingWindow<T> > windower_ptr = this->windower.lock();

    if ( !windower_ptr)
        return false;
    
    windower_ptr->update( time );

    // Lock the windower
    windower_ptr->mutex.lock(); 
    
    // Associate the buffered window
    //this->buffered_window = windower_ptr->window;

    // Find the element with the closest time to *now*
    //typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it = std::lower_bound( this->buffered_window.begin(), this->buffered_window.end(), time, _pair_comparator );
    typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it = std::lower_bound( windower_ptr->window.begin(), 
            windower_ptr->window.end(), 
            time, 
            _pair_comparator );

    //if ( it == this->buffered_window.end() ) // This, is bad - can't find the appropriate time
    if ( it == windower_ptr->window.end() ) // This, is bad - can't find the appropriate time
    {
        std::cout.precision(15);
        //std::cout << __PRETTY_FUNCTION__ << time << "->" << this->buffered_window.front().first << ":" << this->buffered_window.back().first << std::endl;
        std::cout << __PRETTY_FUNCTION__ << time << "->" << windower_ptr->window.front().first << ":" << windower_ptr->window.back().first << std::endl;
        return false; 
    }

    double data_swathe_length = 0;

    //typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it_back_iterator = it;
    typename Iterator<T>::WINDOW_ITERATOR it_back_iterator = it;

    L3::WriteLock lock( this->mutex );
        
    this->window.clear();

    // Copy, as we alter the swathe_length variable
    double swathe_length_local = swathe_length;

    // Working backwards, build up the data swathe
    while( data_swathe_length < swathe_length_local )
    {
        // At the beginning? Is this all we have?
        //if ( it_back_iterator == this->buffered_window.begin() )
        if ( it_back_iterator == windower_ptr->window.begin() )
            break; 

        //this->window.push_front( *it_back_iterator );
        this->window.push_front( std::make_pair( it_back_iterator->first, boost::make_shared<T>( *(it_back_iterator->second) )  ) );

        // Compute dt
        data_swathe_length = (*it).first - (*it_back_iterator).first ;
       
        // Continue
        it_back_iterator--;
    }
    windower_ptr->mutex.unlock(); 
    
#ifndef NDEBUG
    if( !this->window.empty() )
        if( time - this->window.back().first > 1.0/50.0 )
            std::cerr << "WHAT" <<  time - this->window.back().first << std::endl;
#endif

    lock.unlock();
    
    return true;
}

} // L3

// Explicit Instantiations
template class L3::ConstantTimeIterator<L3::SMVelocity>;
template class L3::ConstantTimeIterator<L3::SE3>;
template class L3::ConstantTimeIterator<L3::LHLV>;
template class L3::ConstantTimeIterator<L3::LMS151>;

//template void L3::Iterator<L3::LHLV>::getWindow( std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > >& window) ;
//template void L3::Iterator<L3::SE3>::getWindow(std::deque<std::pair<double, boost::shared_ptr<L3::SE3> >, std::allocator<std::pair<double, boost::shared_ptr<L3::SE3> > > >&);
//template void L3::Iterator<L3::LMS151>::getWindow(std::deque<std::pair<double, boost::shared_ptr<L3::LMS151> >, std::allocator<std::pair<double, boost::shared_ptr<L3::LMS151> > > >&);
