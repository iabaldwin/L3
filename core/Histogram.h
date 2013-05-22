#ifndef L3_HISTOGRAM_H
#define L3_HISTOGRAM_H

#include <gsl/gsl_multimin.h>
#include <gsl/gsl_histogram.h>
#include <gsl/gsl_histogram2d.h>

#include "PointCloud.h"

#include <boost/math/special_functions/log1p.hpp>

#include <numeric>

namespace L3
{
    /*
     *  Histogram structure
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

            float               x_delta, y_delta;
            unsigned int        x_bins, y_bins;
            float               x_lower,x_upper,x_centre;
            float               y_lower,y_upper,y_centre;
            gsl_histogram2d*    hist;

            virtual ~Histogram()
            {
                if( hist )
                    gsl_histogram2d_free( hist );
            }

            bool empty() const
            {
                return ((this->x_bins == 0) || (this->y_bins == 0 ) || (this->x_centre == 0 ) || ( this->y_centre ==0 ) );
            }

            void print() const
            {
                std::cout << 
                        this->x_centre << " " << 
                        this->x_lower << " "  << 
                        this->x_upper<< " "  << 

                        this->y_centre<< " "  << 
                        this->y_lower<< " "  << 
                        this->y_upper<< " "  << 

                        this->x_bins<< " "  << 
                        this->y_bins<< std::endl;
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
                if ( ( x_bins == 0 ) || (y_bins == 0 ) )
                {
                    std::cerr << "Erroneous parameters! " << std::endl;
                    exit(-1); 
                }

                if ( hist ) // Re-create
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

            inline double bin( size_t x, size_t y  ) const
            {
                return (double)gsl_histogram2d_get(hist,x,y); 
            }

            std::pair<float, float> coords( size_t x, size_t y  ) const
            {
                if ( !hist )
                    return std::make_pair( 0, 0 );
                else
                    return std::make_pair( hist->xrange[x], hist->yrange[y] );
            }

            double normalizer() const
            {
                return gsl_histogram2d_sum( hist );
            }

            double max() const
            {
                return gsl_histogram2d_max_val( hist );
            }


            double min() const
            {
                return gsl_histogram2d_min_val( hist );
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

            ~HistogramUniformDistance()
            {
                gsl_histogram2d_free( this->hist );
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
        bool copy( Histogram<T>* src, Histogram<T>* dest )
        {
            if ( src->empty() )
                return false;
            
            L3::WriteLock(dest->mutex);
            dest->create( src->x_centre,     
                    src->x_lower,
                    src->x_upper,
                    src->y_centre,
                    src->y_lower,
                    src->y_upper,      
                    src->x_bins,
                    src->y_bins 
                    );

            return true;
        }

    template <typename T>
        std::ostream& operator<<( std::ostream& o, const Histogram<T>& h );

    template <typename T>
        void clone( Histogram<T>* src, Histogram<T> * dest );

    template <typename T>
        struct HistogramPyramid : Lockable
        {
            HistogramPyramid( std::vector<T> densities )
            {
                for( typename std::vector<T>::iterator it=densities.begin(); 
                        it !=densities.end();
                        it++) 
                    this->histograms.push_back( 
                            boost::make_shared<L3::HistogramUniformDistance<T> > ( 
                                *it ) );

            }

            std::deque< boost::shared_ptr< Histogram<T> > > histograms;

            typedef typename std::deque< boost::shared_ptr< Histogram<T> > >::iterator  PYRAMID_ITERATOR;

            PYRAMID_ITERATOR begin()
            {
                return histograms.begin();
            }

            PYRAMID_ITERATOR end()
            {
                return histograms.end();
            }

            size_t size()
            {
                return histograms.size();     
            }

            boost::shared_ptr< Histogram<T> > operator[]( unsigned int index )
            {
                return histograms[index];
            }

            void print()
            {
                for( PYRAMID_ITERATOR it = this->begin();
                        it != this->end();
                        it++ )
                {
                    L3::ReadLock( (*it)->mutex );
                    (*it)->print();
                }
            }

        };


    double compute_entropy( gsl_histogram* histogram );
    double compute_entropy( gsl_histogram2d* histogram );
    

}

#endif

