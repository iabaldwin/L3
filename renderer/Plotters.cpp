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

            L3::ReadLock velocity_reader( iterator_ptr->mutex );

            if (filtered)
                window.assign( iterator_ptr->filtered_velocities.begin(), iterator_ptr->filtered_velocities.end() );
            else
            {
                window.assign( iterator_ptr->raw_velocities.begin(), iterator_ptr->raw_velocities.end() );
                this->color ( glv::Color( .5, .5, .5 ) );
                this->mPrim = glv::draw::Points;
            }
          
            velocity_reader.unlock();


            glv::Data tmp;
            
            if ( window.size() > 0 )
            {
                tmp.resize( glv::Data::DOUBLE, 2, window.size() );

                glv::Indexer i(tmp.size(1));

                int counter = 0;

                // Render 10s
                double zero_time;
                if( time_lock )
                    zero_time = time_lock->t - 10;
                else
                    zero_time = window.back().first;

                while( i() && (unsigned int)counter < window.size() ) 
                {
                    double dt = window[counter].first - zero_time;
                    double variable = window[counter].second[index];

                    tmp.assign( dt, 0, counter );
                    tmp.assign( variable, 1, counter );

                    counter++;
                }

                //x_limit = window.back().first - window.front().first;
                x_limit = 10.0;
                //y_centroid = std::max( y_centroid, variable );
                y_centroid = std::max( window.front().second[index] ,window.back().second[index] );


            }
            
            L3::WriteLock writer( this->mutex );
            mData = tmp; 
            writer.unlock();

            boost::shared_ptr< glv::View > parent = plot_parent.lock();

            if( parent )
            {
                boost::shared_ptr< glv::Plot > parent_plot = boost::dynamic_pointer_cast< glv::Plot >( parent );

                if ( parent_plot )
                {
                    parent_plot->range( 0, x_limit + 0.5 , 0 );
                    parent_plot->range( -1, y_centroid + 2.0 , 1 );
                }
            }
        }
    }
}
