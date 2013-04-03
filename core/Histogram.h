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
            float delta;
            unsigned int num_bins;

            Histogram(  float x_centre=0.f, 
                    float x_lower=50.0f, 
                    float x_upper=50.0f, 
                    float y_centre=0.f, 
                    float y_lower=50.0f, 
                    float y_upper=50.0f, 
                    unsigned int bins=1000 ) : num_bins(bins)
            {
                hist =  gsl_histogram2d_alloc (num_bins, num_bins);

                gsl_histogram2d_set_ranges_uniform (hist, 
                        x_centre-x_lower, 
                        x_centre+x_upper, 
                        y_centre-y_lower, 
                        y_centre+y_upper );

                delta = 100.0/num_bins;
            }
           
            //Raw Histogram 
            gsl_histogram2d* hist;

            ~Histogram()
            {
                gsl_histogram2d_free( hist );
            }

            void reset()
            {
                gsl_histogram2d_reset( hist );
            }

            unsigned int bin( size_t x, size_t y  )
            {
                return gsl_histogram2d_get(hist,x,y); 
            }

            std::pair<float, float> coords( size_t x, size_t y  )
            {
                return std::make_pair( hist->xrange[x], hist->yrange[y] );
            }

            void operator()( L3::PointCloud<T>* cloud )
            {
                for( typename L3::PointCloud<T>::ITERATOR it = cloud->begin(); it != cloud->end(); it++ )
                    gsl_histogram2d_increment( hist, it->x, it->y );
            }

        };

}

#endif
