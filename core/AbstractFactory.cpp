#include "AbstractFactory.h"

namespace L3
{

/*
 *  Masking policy
 */
boost::shared_ptr<L3::LMS151> MaskPolicy<L3::LMS151>::operator()( boost::shared_ptr<L3::LMS151> t ) 
{
    for( std::vector<float>::iterator it = t->ranges.begin(); 
            it != t->ranges.end();
            it++ )
    {
        if ( *it < cull )
            *it = 0.0;
    }

    return t;
}


/*
 *  Base type
 */
template <typename T>
std::pair< double, boost::shared_ptr<T> > AbstractFactory<T>::produce( std::string& str, MaskPolicy<T> mask_policy )
{
    std::stringstream ss( str );
    std::vector< double > elements;

    double tmp;

    while ( ss >> tmp )
        elements.push_back( tmp ); 

    return AbstractFactory<T>::produce( elements, mask_policy );
}

template <typename T>
std::pair< double, boost::shared_ptr<T> > AbstractFactory<T>::produce( std::vector<double> elements, MaskPolicy<T> mask_policy )
{
    assert( elements.size() > 0 );

    double time = elements[0];
    elements.erase( elements.begin() );

    return std::make_pair( time, mask_policy( boost::make_shared<T>( elements ) ) );
}


/*
 *  Pose
 */
std::pair< double, boost::shared_ptr<L3::Pose> >  AbstractFactory<L3::Pose>::produce( std::string& str )
{
    std::stringstream ss( str );
    std::vector< double > elements;
    
    double tmp;
    
    while ( ss >> tmp )
        elements.push_back( tmp ); 

    return AbstractFactory<L3::Pose>::produce( elements );
}

std::pair< double, boost::shared_ptr<L3::Pose> > AbstractFactory<L3::Pose>::produce( std::vector<double> elements )
{
    assert( elements.size() > 0 );
    
    double time = elements[0];
    elements.erase( elements.begin() );

    switch (elements.size())
    {
        case 3:
            return std::make_pair( time, boost::make_shared<L3::SE2>( elements ) );

        case 6:
            return std::make_pair( time, boost::make_shared<L3::SE3>( elements ) );

        default:
            throw std::exception();
    }
}

} // L3

// Explicit instantiations
template class L3::AbstractFactory<L3::SMVelocity>;
template class L3::AbstractFactory<L3::LHLV>;
template class L3::AbstractFactory<L3::LMS151>;
template std::pair< double, boost::shared_ptr<L3::SE3> > L3::AbstractFactory<L3::SE3>::produce(std::basic_string<char, std::char_traits<char>, std::allocator<char> >&, L3::MaskPolicy<L3::SE3>);
template std::pair< double, boost::shared_ptr<L3::SE3> > L3::AbstractFactory<L3::SE3>::produce(std::vector< double, std::allocator<double> >, L3::MaskPolicy<L3::SE3>);


