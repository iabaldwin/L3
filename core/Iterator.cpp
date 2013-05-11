#include "Iterator.h"
#include "Misc.h"

namespace L3
{

template <typename T>
void Iterator<T>::getWindow( typename std::deque< std::pair< double, boost::shared_ptr<T> > >& window)  
{
    mutex.lock();
    
    window.resize( this->window.size() );
    std::copy( this->window.begin(), this->window.end(), window.begin() );
    
    mutex.unlock();
}

template <typename T>
bool ConstantTimeIterator<T>::update( double time )
{
    // Update the watcher with the new time
    boost::shared_ptr< L3::SlidingWindow<T> > windower_ptr = this->windower.lock();

    if ( !windower_ptr)
        return false;
    
    windower_ptr->update( time );

    // Retrive the buffered window
    this->buffered_window = windower_ptr->getWindow();

    // Find the element with the closest time to *now*
    typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it = std::lower_bound( this->buffered_window.begin(), this->buffered_window.end(), time, _pair_comparator );

    if ( it == this->buffered_window.end() ) // This, is bad - can't find the appropriate time
    {
        std::cout.precision(15);
        std::cout << __PRETTY_FUNCTION__ << time << "->" << this->buffered_window.front().first << ":" << this->buffered_window.back().first << std::endl;
        //L3::Misc::print_stack_trace();
        return false; 
    }

    it = ( fabs(( it-1 )->first - time)  ) < ( fabs(( it )->first - time)  ) ? ( it-1) : it;

    double data_swathe_length = 0;

    typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it_back_iterator = it;

    this->mutex.lock();
    this->window.clear();
   
    // Working backwards, build up the data swathe
    while( data_swathe_length < swathe_length )
    {
        // At the beginning? Is this all we have?
        if ( it_back_iterator == this->buffered_window.begin() )
            break; 

        this->window.push_front( *it_back_iterator );

        // Compute dt
        data_swathe_length = (*it).first - (*it_back_iterator).first ;
       
        // Continue
        it_back_iterator--;
    }
    
    this->mutex.unlock();

    return true;
}

} // L3

// Explicit Instantiations
template class L3::ConstantTimeIterator<L3::SE3>;
template class L3::ConstantTimeIterator<L3::LHLV>;
template class L3::ConstantTimeIterator<L3::LMS151>;

template void L3::Iterator<L3::LHLV>::getWindow( std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > >& window) ;
template void L3::Iterator<L3::SE3>::getWindow(std::deque<std::pair<double, boost::shared_ptr<L3::SE3> >, std::allocator<std::pair<double, boost::shared_ptr<L3::SE3> > > >&);
template void L3::Iterator<L3::LMS151>::getWindow(std::deque<std::pair<double, boost::shared_ptr<L3::LMS151> >, std::allocator<std::pair<double, boost::shared_ptr<L3::LMS151> > > >&);
