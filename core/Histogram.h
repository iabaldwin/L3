#ifndef L3_HISTOGRAM_H
#define L3_HISTOGRAM_H

#include <gsl/gsl_histogram2d.h>

#include "PointCloud.h"

namespace L3
{
    /*
     * Statistics
     */
    template <typename T>

        struct Histogram : Lockable
        {

            Histogram() : x_delta(0.0), y_delta(0.0), x_bins(0), y_bins(0), 
            x_lower(.0f),x_upper(.0f),x_centre(.0f),
            y_lower(.0f),y_upper(.0f),y_centre(.0f),
            hist(NULL)
            {
            }

            float               x_lower,x_upper,x_centre;
            float               y_lower,y_upper,y_centre;
            float               x_delta, y_delta;
            unsigned int        x_bins, y_bins;
            gsl_histogram2d*    hist;

            virtual ~Histogram()
            {
                gsl_histogram2d_free( hist );
            }


            virtual void create( float x_centre,
                    float x_lower,
                    float x_upper,
                    float y_centre,
                    float y_lower,
                    float y_upper,
                    unsigned int x_bins,
                    unsigned int y_bins )
            {
                if ( hist )
                    gsl_histogram2d_free( hist );

                this->x_centre= x_centre;
                this->x_lower = x_lower;
                this->x_upper = x_upper;

                this->y_centre= y_centre;
                this->y_lower = y_lower;
                this->y_upper = y_upper;

                this->x_bins = x_bins;
                this->y_bins = y_bins;


                // Allocate histogram
                hist =  gsl_histogram2d_alloc ( x_bins, y_bins );

                // Set ranges
                gsl_histogram2d_set_ranges_uniform (hist, 
                        x_lower, 
                        x_upper, 
                        y_lower, 
                        y_upper );

                // Compute delta
                x_delta = (x_upper - x_lower)/x_bins;
                y_delta = (y_upper - y_lower)/y_bins;

            }

            void clear()
            {
                gsl_histogram2d_reset( hist );
            }

            unsigned int bin( size_t x, size_t y  )
            {
                return gsl_histogram2d_get(hist,x,y); 
            }

            std::pair<float, float> coords( size_t x, size_t y  )
            {
                if ( !hist )
                    return std::make_pair( 0, 0 );
                else
                    return std::make_pair( hist->xrange[x], hist->yrange[y] );
            }

            // Histogram an entire cloud
            void operator()( L3::PointCloud<T>* cloud )
            {
                for( typename L3::PointCloud<T>::ITERATOR it = cloud->begin(); it != cloud->end(); it++ )
                    gsl_histogram2d_increment( hist, it->x, it->y );
            }

        };

    template <typename T>
        struct HistogramUniformDistance : Histogram<T>
        {
            HistogramUniformDistance( float bins_per_metre=1.0f ) : bins_per_metre(bins_per_metre)
            {

            }

            float bins_per_metre;

            void create( float x_centre,
                    float x_lower,
                    float x_upper,
                    float y_centre,
                    float y_lower,
                    float y_upper )
            {
                Histogram<T>::create( x_centre, x_lower, x_upper, y_centre, y_lower, y_upper, bins_per_metre*( x_upper - x_lower ), bins_per_metre*( y_upper - y_lower ) );
            }

        };


    template <typename T>
        void copy( Histogram<T> const* src, Histogram<T> * dest )
        {
            dest->create( src->x_centre,     
                    src->x_lower,
                    src->x_upper,
                    src->y_centre,
                    src->y_lower,
                    src->y_upper,      
                    src->x_bins,
                    src->y_bins 
                    );

        }

    template <typename T>
        std::ostream& operator<<( std::ostream& o, const Histogram<T>& h );
}

#endif

