#ifndef L3_VISUALISERS_PLOTTERS_H
#define L3_VISUALISERS_PLOTTERS_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

namespace L3
{
    namespace Visualisers
    {

        struct VelocityPlotter : glv::Plottable, TemporalObserver
        {
            VelocityPlotter( L3::ConstantTimeIterator< L3::LHLV >* lhlv_iterator )
                : glv::Plottable( glv::draw::LineStrip, 1, glv::Color(0)),
                    iterator(lhlv_iterator)
            {
                index = -1;
                    
                data.resize( glv::Data::DOUBLE, 1, 10000 );
            }

            virtual ~VelocityPlotter()
            {
            }

            int                                     index;
            glv::Data                               data;
            L3::ConstantTimeIterator< L3::LHLV>*    iterator; 

            void onMap( glv::GraphicsData& b, const glv::Data& d, const glv::Indexer& i){
                std::cout << "BYE" << std::endl;
            }

            bool update( double )
            {
                std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > window ;

                iterator->getWindow( window );

                if ( window.size() > 0 )
                {
                    data.resize( glv::Data::DOUBLE, 1, window.size() );

                    glv::Indexer i(data.size(1));

                    int counter = 0;

                    while( i() && counter < window.size() ) 
                    {
                        double d = window[counter++].second->data[index];
                        data.assign( d, i[0], i[1] );
                    }

                    // This is the issue
                    //plotter->data() = data;
                }
            }

            void onDraw2D( glv::GLV& g )
            {
                std::cout << "BYE" << std::endl;
            }
            void onDraw( glv::GLV& g )
            {
                std::cout << "BYE" << std::endl;
            }

            void onDraw( glv::GraphicsData& gd, const glv::Data& d )
            {
                std::cout << "HI" << std::endl;
            }

        };

        //struct LinearVelocityPlotter : VelocityPlotter
        //{
            //LinearVelocityPlotter(L3::ConstantTimeIterator< L3::LHLV >* lhlv_iterator , glv::PlotFunction1D* plotter ) :
                //VelocityPlotter( lhlv_iterator, plotter )
            //{
                //index = 9;
            //}
        //};

        //struct RotationalVelocityPlotter : VelocityPlotter
        //{
            //RotationalVelocityPlotter(L3::ConstantTimeIterator< L3::LHLV >* lhlv_iterator , glv::PlotFunction1D* plotter ) :
                //VelocityPlotter( lhlv_iterator, plotter )
            //{
                //index = 3;
            //}
        //};

    } // Visualisers
} // L3

#endif
