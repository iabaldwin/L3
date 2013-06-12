#include "Plotters.h"

namespace L3
{
    namespace Visualisers
    {
        void VelocityPlotter::update()
        {
                boost::shared_ptr< L3::VelocityProvider > iterator_ptr = iterator.lock();

                if( !iterator_ptr)
                    return;

                VELOCITY_WINDOW window;
                
                if (filtered)
                    window.assign( iterator_ptr->filtered_velocities.begin(), iterator_ptr->filtered_velocities.end() );
                else
                {
                    window.assign( iterator_ptr->raw_velocities.begin(), iterator_ptr->raw_velocities.end() );
                    this->color ( glv::Color( .5, .5, .5 ) );
                    this->mPrim = glv::draw::Points;
                }

                if ( window.size() > 0 )
                {
                    L3::WriteLock writer( this->mutex );
                   
                    mData.resize( glv::Data::DOUBLE, 2, window.size() );

                    glv::Indexer i(mData.size(1));

                    int counter = 0;

                    // Render 10s
                    double zero_time;
                    if( time_lock )
                        zero_time = time_lock->t - 10;
                    else
                        zero_time = window.back().first;

                    while( i() && counter < window.size() ) 
                    {
                        double dt = window[counter].first - zero_time;
                        double variable = window[counter].second[index];

                        mData.assign( dt, 0, counter );
                        mData.assign( variable, 1, counter );
                      
                        counter++;
                    }

                    //x_limit = window.back().first - window.front().first;
                    x_limit = 10.0;

                        
                    //y_centroid = std::max( y_centroid, variable );

                    y_centroid = window.back().second[index];


                    writer.unlock();
                }

                boost::shared_ptr< glv::View > parent = plot_parent.lock();
               
                if( parent )
                {
                    boost::shared_ptr< glv::Plot > parent_plot = boost::dynamic_pointer_cast< glv::Plot >( parent );
                
                    if ( parent_plot )
                    {
                        parent_plot->range( 0, x_limit + 0.5 , 0 );
                        parent_plot->range( y_centroid -2.0, y_centroid + 2.0 , 1 );
                    }
                }
               
            }

    }

}
