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
        
        struct Histogram
        {

            Histogram() : x_delta(0.0), y_delta(0.0), x_bins(0), y_bins(0), hist(NULL)
            {
            }
           
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
                
                // Allocate histogram
                hist =  gsl_histogram2d_alloc ( x_bins, y_bins );

                // Set ranges
                gsl_histogram2d_set_ranges_uniform (hist, 
                                                    x_centre+x_lower, 
                                                    x_centre+x_upper, 
                                                    y_centre+y_lower, 
                                                    y_centre+y_upper );

                // Compute delta
                x_delta = 100.0/x_bins;
                y_delta = 100.0/y_bins;

                this->x_bins = x_bins;
                this->y_bins = y_bins;


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
            float bins_per_metre;

            void create( float x_centre,
                            float x_lower,
                            float x_upper,
                            float y_centre,
                            float y_lower,
                            float y_upper,
                            float  bins_per_metre )
            {
                Histogram<T>::create( x_centre, x_lower, x_upper, y_centre, y_lower, y_upper, bins_per_metre*( x_upper - x_lower ), bins_per_metre*( y_upper - y_lower ) );
            }

        };
 
}

#endif

