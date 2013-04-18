#ifndef L3_VISUALISERS_PLOTTERS_H
#define L3_VISUALISERS_PLOTTERS_H

namespace L3
{
    namespace Visualisers
    {

        struct Plotter  : TemporalObserver
        {
        };

        struct VelocityPlotter : Plotter
        {

            VelocityPlotter( L3::ConstantTimeIterator< L3::LHLV >* lhlv_iterator , glv::PlotFunction1D* plotter ) 
                : iterator(lhlv_iterator), 
                    plotter( plotter )

            {
                index = -1;
            }

            virtual ~VelocityPlotter()
            {
            }

            int                                     index;
            glv::Data                               data;
            glv::PlotFunction1D*                    plotter;
            L3::ConstantTimeIterator< L3::LHLV>*    iterator; 

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
               
                    plotter->data() = data;
                }
            }
        };

        struct LinearVelocityPlotter : VelocityPlotter
        {
            LinearVelocityPlotter(L3::ConstantTimeIterator< L3::LHLV >* lhlv_iterator , glv::PlotFunction1D* plotter ) :
                VelocityPlotter( lhlv_iterator, plotter )
            {
                index = 9;
            }
        };

        struct RotationalVelocityPlotter : VelocityPlotter
        {
            RotationalVelocityPlotter(L3::ConstantTimeIterator< L3::LHLV >* lhlv_iterator , glv::PlotFunction1D* plotter ) :
                VelocityPlotter( lhlv_iterator, plotter )
            {
                index = 3;
            }
        };



    } // Visualisers
} // L3

#endif
