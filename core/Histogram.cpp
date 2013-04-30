#include "Histogram.h"

namespace L3
{
    template <typename T>
        std::ostream& operator<<( std::ostream& o, const Histogram<T>& h )
        {
            o << h.x_centre  << ", "      << 
                h.x_lower   << ", "<< 
                h.x_upper   << ", "<< 
                h.y_centre  << ", "<< 
                h.y_lower   << ", "<< 
                h.y_upper   << ", "  <<   
                h.x_bins    << ", " << 
                h.y_bins << std::endl;

            o << "X Edges:" << std::endl;
            std::copy( h.hist->xrange, h.hist->xrange+h.x_bins, std::ostream_iterator<double>( o, " " ) ); 
            o << std::endl; 

            o << "Y Edges:" << std::endl;
            std::copy( h.hist->yrange, h.hist->yrange+h.y_bins, std::ostream_iterator<double>( o, " " ) ); 
            o << std::endl; 

            o << "Counts:" << std::endl;
            std::copy( h.hist->bin, h.hist->bin+h.x_bins*h.y_bins, std::ostream_iterator<double>( o, " " ) ); 

            return o;
        }

    template <typename T>
        void clone( Histogram<T>* src, Histogram<T> * dest )
        {
            L3::WriteLock(dest->mutex);

            // Parameter copy
            dest->create( src->x_centre,     
                    src->x_lower,
                    src->x_upper,
                    src->y_centre,
                    src->y_lower,
                    src->y_upper,      
                    src->x_bins,
                    src->y_bins 
                    );


            // Data copy
            gsl_histogram2d_memcpy( dest->hist, src->hist );
        }
}

template std::ostream& L3::operator<<( std::ostream& o, const Histogram<double>& h );
template void L3::clone<double>(L3::Histogram<double>*, L3::Histogram<double>*);
