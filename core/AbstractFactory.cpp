#include "AbstractFactory.h"


namespace L3
{

/*
 *Base type
 */
template <typename T>
std::pair< double, boost::shared_ptr<T> > AbstractFactory<T>::produce( std::string& str )
{
    std::stringstream ss( str );
    std::vector< double > elements;

    double tmp;

    while ( ss >> tmp )
    {
        elements.push_back( tmp ); 
    }


    return AbstractFactory<T>::produce( elements );
}

template <typename T>
std::pair< double, boost::shared_ptr<T> > AbstractFactory<T>::produce( std::vector<double> elements )
{
    assert( elements.size() > 0 );

    double time = elements[0];
    elements.erase( elements.begin() );

    return std::make_pair( time, boost::shared_ptr<T>( new T( elements ) ) );
}

std::pair< double, boost::shared_ptr<L3::SE3> > AbstractFactory<L3::SE3>::produce( std::string& str )
{
    std::stringstream ss( str );
    std::vector< double > elements;
    
    double tmp;
    
    while ( ss >> tmp )
    {
        elements.push_back( tmp ); 
    }


    return AbstractFactory<L3::SE3>::produce( elements );
}
        
std::pair< double, boost::shared_ptr<L3::SE3> >  AbstractFactory<L3::SE3>::produce( std::vector<double> elements )
{
    assert( elements.size() > 0 );

    double time = elements[0];
    elements.erase( elements.begin() );

    return std::make_pair( time, boost::shared_ptr<L3::SE3>( new L3::SE3( elements ) ) );
}

std::pair< double, boost::shared_ptr<L3::LIDAR> >  AbstractFactory<L3::LIDAR>::produce( std::string& str )
{
    std::stringstream ss( str );
    std::vector< double > elements;

    double tmp;
    
    while ( ss >> tmp )
    {
        elements.push_back( tmp ); 
    }
    
    return AbstractFactory<L3::LIDAR>::produce( elements );
}

std::pair< double, boost::shared_ptr<L3::LIDAR> > AbstractFactory<L3::LIDAR>::produce( std::vector<double> elements )
{
    assert( elements.size() > 0 );

    double time = elements[0];
    elements.erase( elements.begin() );
    switch (elements.size())
    {
        case 541:
            return std::make_pair( time, boost::shared_ptr<L3::LIDAR>( new LMS151( elements ) ) );

        default:
            throw std::exception();

    }

}

std::pair< double, boost::shared_ptr<L3::Pose> >  AbstractFactory<L3::Pose>::produce( std::string& str )
{
    std::stringstream ss( str );
    std::vector< double > elements;
    
    double tmp;
    
    while ( ss >> tmp )
    {
        elements.push_back( tmp ); 
    }

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
            return std::make_pair( time, boost::shared_ptr<L3::SE2>( new L3::SE2( elements ) ) );

        case 6:
            return std::make_pair( time, boost::shared_ptr<L3::SE3>( new L3::SE3( elements ) ) );

        default:
            std::copy( elements.begin(), 
                    elements.end(), 
                    std::ostream_iterator<double>( std::cout, " " ) );
            throw std::exception();
    }
}

} // L3

// Explicit instantiations
template class L3::AbstractFactory<L3::LMS151>;
template class L3::AbstractFactory<L3::LHLV>;

