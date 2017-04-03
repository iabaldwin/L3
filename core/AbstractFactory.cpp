#include "AbstractFactory.h"

namespace L3
{
     
/**
 * @brief Masking policy for incoming data segments
 *
 * @param t data type
 *
 * @return Masked data
 */
boost::shared_ptr<L3::LMS151> MaskPolicy<L3::LMS151>::operator()( boost::shared_ptr<L3::LMS151> t ) 
{
    for( std::vector<float>::iterator it = t->ranges.begin(); 
            it != t->ranges.end();
            it++ )
    {
        if ( *it < this->cull )
            *it = 0.0;
    }

    return t;
}

/**
 * @brief Produce a time-stamped data element from a raw string, using masking policy
 *
 * @tparam T            Templated data type
 * @param str           Incoming string
 * @param mask_policy   Masking policy
 *
 * @return              Time-stamped type
 */
template <typename T>
    std::pair< double, boost::shared_ptr<T> > AbstractFactory<T>::produce( std::string& str, MaskPolicy<T>* mask_policy )
    {
        std::stringstream ss( str );
        std::vector< double > elements;

        double tmp;

        while ( ss >> tmp )
            elements.push_back( tmp ); 

        return AbstractFactory<T>::produce( elements, mask_policy );
    }

/**
 * @brief Produce a time-stamped data element from a vector of numeric data, using masking policy
 *
 * @tparam T            Templated data type
 * @param elements      Incoming (numeric) data
 * @param mask_policy   Masking policy
 *
 * @return              Time-stamped type
 */
template <typename T>
    std::pair< double, boost::shared_ptr<T> > AbstractFactory<T>::produce( std::vector<double> elements, MaskPolicy<T>* mask_policy )
    {
        assert( elements.size() > 0 );

        double time = elements[0];
        elements.erase( elements.begin() );

        if( mask_policy )
            return std::make_pair( time, (*mask_policy)( boost::make_shared<T>( elements ) ) );
        else 
            return std::make_pair( time, boost::make_shared<T>( elements ) );
    }

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
template std::pair< double, boost::shared_ptr<L3::SE3> > L3::AbstractFactory<L3::SE3>::produce(std::basic_string<char, std::char_traits<char>, std::allocator<char> >&, L3::MaskPolicy<L3::SE3>* );
template std::pair< double, boost::shared_ptr<L3::SE3> > L3::AbstractFactory<L3::SE3>::produce(std::vector< double, std::allocator<double> >, L3::MaskPolicy<L3::SE3>* );
template std::pair< double, boost::shared_ptr<L3::SMVelocity> > L3::AbstractFactory<L3::SMVelocity>::produce(std::basic_string<char, std::char_traits<char>, std::allocator<char> >&, L3::MaskPolicy<L3::SMVelocity>*);
template std::pair< double, boost::shared_ptr<L3::LHLV> >  L3::AbstractFactory<L3::LHLV>::produce(std::basic_string<char, std::char_traits<char>, std::allocator<char> >&, L3::MaskPolicy<L3::LHLV>*);
template std::pair< double, boost::shared_ptr<L3::LMS151> > L3::AbstractFactory<L3::LMS151>::produce(std::basic_string<char, std::char_traits<char>, std::allocator<char> >&, L3::MaskPolicy<L3::LMS151>*);
