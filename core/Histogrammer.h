#ifndef L3_HISTOGRAMMER_H
#define L3_HISTOGRAMMER_H

#include <gsl/gsl_histogram2d.h>

#include "PointCloud.h"

namespace L3
{
    /*
     * Statistics
     */
    template <typename T>
        
        struct histogram
        {
            float delta;
            unsigned int num_bins;

            histogram(  float x_centre=0.f, 
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
            
            gsl_histogram2d* hist;

            ~histogram()
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


    template <typename T>
        struct Histogrammer
        {

            Histogrammer( L3::PointCloud<T>* EXPERIENCE, L3::PointCloud<T>* SWATHE ) :
                experience(EXPERIENCE), 
                swathe(SWATHE),
                granularity(40)
            {
            }

            L3::PointCloud<T>* experience;      // Experience swathe, localising *against* this
            L3::PointCloud<T>* swathe ;         // Run-time swathe, localising *with8 this
            int granularity;

            void histogram()
            {
                std::pair<T,T> min_bound = L3::min<T>( experience );
                std::pair<T,T> max_bound = L3::max<T>( experience );
                std::pair<T,T> means     = L3::mean( experience );

                // Build experience histogram 
                L3::histogram<T> experience_hist( means.first, 
                                                    means.first - min_bound.first, 
                                                    max_bound.first - means.first, 
                                                    means.second, 
                                                    means.second - min_bound.second, 
                                                    max_bound.second - means.second, 
                                                    granularity );

        
                L3::histogram<T> swathe_hist( means.first, 
                                                    means.first - min_bound.first, 
                                                    max_bound.first - means.first, 
                                                    means.second, 
                                                    means.second - min_bound.second, 
                                                    max_bound.second - means.second, 
                                                    granularity );

                experience_hist( experience );
                swathe_hist( swathe );
            }

        };

}

#endif

