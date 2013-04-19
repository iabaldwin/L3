#ifndef L3_VISUALISERS_PLOTTERS_H
#define L3_VISUALISERS_PLOTTERS_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

namespace L3
{
    namespace Visualisers
    {
        struct VelocityPlotter : glv::Plottable, TemporalObserver, Lockable
        {
            VelocityPlotter( L3::ConstantTimeIterator< L3::LHLV >* lhlv_iterator )
                : glv::Plottable( glv::draw::LineStrip, 1 ),
                    iterator(lhlv_iterator), 
                    index(-1)
            {
            }

            virtual ~VelocityPlotter()
            {
            }

            int                                     index;
            L3::ConstantTimeIterator< L3::LHLV>*    iterator; 

            void onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i)
            {
                L3::ReadLock( this->mutex );

                while(i()){
                    double x = i[0];
                    double y = d.at<double>(0, i[0]);
                    g.addVertex(x, y);
                }
            }

            bool update( double )
            {
                std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > window ;

                iterator->getWindow( window );

                if ( window.size() > 0 )
                {
                    L3::WriteLock( this->mutex );
                   
                    mData.resize( glv::Data::DOUBLE, 1, window.size() );

                    std::cout << "Resize:" << window.size() << std::endl;

                    glv::Indexer i(mData.size(1));

                    int counter = 0;

                    while( i() && counter < window.size() ) 
                    {
                        double d = window[counter++].second->data[index];
                        mData.assign( d, i[0], i[1] );
                    }

                }
            }

        };

        struct LinearVelocityPlotter : VelocityPlotter
        {
            LinearVelocityPlotter(L3::ConstantTimeIterator< L3::LHLV >* lhlv_iterator ) :
                VelocityPlotter( lhlv_iterator )
            {
                index = 9;
                this->color( glv::Color( 1,0,0 ) );
            }
        };

        struct RotationalVelocityPlotter : VelocityPlotter
        {
            RotationalVelocityPlotter(L3::ConstantTimeIterator< L3::LHLV >* lhlv_iterator ) :
                VelocityPlotter( lhlv_iterator )
            {
                index = 3;
                this->color( glv::Color( 0,1,0 ) );
            }
        };

    } // Visualisers
} // L3

#endif
