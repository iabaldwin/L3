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
                window.assign( iterator_ptr->velocities.begin(), iterator_ptr->velocities.end() );

                y_limit = -1*std::numeric_limits<double>::infinity();

                if ( window.size() > 0 )
                {
                    L3::WriteLock writer( this->mutex );
                   
                    mData.resize( glv::Data::DOUBLE, 2, window.size() );

                    glv::Indexer i(mData.size(1));

                    int counter = 0;

                    // Render 10s
                    //double zero_time = window.back().first - 10.0;
                    double zero_time = window.back().first - 10.0;

                    while( i() && counter < window.size() ) 
                    {
                        //double dt = window[counter].first - window[0].first;
                        double dt = window[counter].first - zero_time;
                        double variable = window[counter].second[index];

                        y_limit = std::max( y_limit, variable );

                        mData.assign( dt, 0, counter );
                        mData.assign( variable, 1, counter );
                      
                        counter++;
                    }

                    x_limit = window.back().first - window.front().first;

                    writer.unlock();
                }

                boost::shared_ptr< glv::View > parent = plot_parent.lock();
               
                if( parent )
                {
                    boost::shared_ptr< glv::Plot > parent_plot = boost::dynamic_pointer_cast< glv::Plot >( parent );
                
                    if ( parent_plot )
                    {
                        parent_plot->range( 0, x_limit + 0.5 , 0 );
                        parent_plot->range( -1, y_limit + 2.0 , 1 );

                        parent_plot->minor( y_tmp, 0 );
                        parent_plot->major( x_tmp, 0 );
                    }
                }
               
            }

    }

}
