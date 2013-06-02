#ifndef L3_VISUALISERS_PLOTTERS_H
#define L3_VISUALISERS_PLOTTERS_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

namespace L3
{
    namespace Visualisers
    {
        struct VelocityPlotter : glv::Plottable, Lockable, Updateable
        {
            VelocityPlotter()
                : glv::Plottable( glv::draw::LineStrip, 1 ),
                    index(-1)
            {
            }

            virtual ~VelocityPlotter()
            {
            }

            int index;
            double current;
            boost::weak_ptr< glv::Plot > plot_parent; 
            boost::weak_ptr< L3::ConstantTimeIterator< L3::LHLV> > iterator; 

            void setParent( boost::shared_ptr< glv::Plot > plotter )
            {
                plot_parent = plotter;
            }
             
            void assignIterator( boost::shared_ptr< L3::ConstantTimeIterator< L3::LHLV > > LHLV_iterator )
            {
                this->iterator = LHLV_iterator;
            }

            //void onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i)
            //{
                //L3::ReadLock( this->mutex );

                ////while(i()){
                    ////double x = i[0];
                    ////double y = d.at<double>(0, i[0]);
                    ////g.addVertex(x, y);
                ////}
            //}

            void update()
            {
                boost::shared_ptr< L3::ConstantTimeIterator< L3::LHLV > > iterator_ptr = iterator.lock();

                if( !iterator_ptr)
                    return;

                std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > window ;

                iterator_ptr->getWindow( window );

                current = -1*std::numeric_limits<double>::infinity();

                if ( window.size() > 0 )
                {
                    L3::WriteLock( this->mutex );
                   
                    mData.resize( glv::Data::DOUBLE, 1, window.size() );

                    glv::Indexer i(mData.size(1));

                    int counter = 0;

                    while( i() && counter < window.size() ) 
                    {
                        double d = window[counter++].second->data[index];

                        if ( d > current )
                            current = d;

                        mData.assign( d, i[0], i[1] );
                    }

                }

                boost::shared_ptr< glv::View > parent = plot_parent.lock();
               
                if( parent )
                {
                    boost::shared_ptr< glv::Plot > parent_plot = boost::dynamic_pointer_cast< glv::Plot >( parent );
                    if ( parent_plot )
                        parent_plot->range( -1, current + 2.0 , 1 );
                }
               
            }

        };

        struct LinearVelocityPlotter : VelocityPlotter
        {
            LinearVelocityPlotter() 
            {
                index = 9;
                this->color( glv::Color( 1,0,0 ) );
            }
        };

        struct RotationalVelocityPlotter : VelocityPlotter
        {
            RotationalVelocityPlotter() 
            {
                index = 3;
                this->color( glv::Color( 0,1,0 ) );
            }
        };

    }   // Visualisers
}       // L3

#endif
