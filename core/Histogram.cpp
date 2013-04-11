#include "Histogram.h"

namespace L3
{

    template <typename T>
    std::ostream& operator<<( std::ostream& o, const Histogram<T>& h )
    {

        o << h.x_centre  <<      
                    h.x_lower   << 
                    h.x_upper   << 
                    h.y_centre  << 
                    h.y_lower   << 
                    h.y_upper   <<       
                    h.x_bins    << 
                    h.y_bins;

        return o;
    }

}


template std::ostream& L3::operator<<( std::ostream& o, const Histogram<double>& h );
