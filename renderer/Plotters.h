#ifndef L3_VISUALISERS_PLOTTERS_H
#define L3_VISUALISERS_PLOTTERS_H

namespace L3
{
namespace Visualisers
{

struct Plotter 
{
};

struct VelocityPlotter : Plotter
{

    VelocityPlotter( L3::ConstantTimeIterator< L3::LHLV>* lhlv_iterator , glv::PlotFunction1D* vel_plotter ) 
        : iterator(lhlv_iterator), plotter( vel_plotter )
    {
    }

    bool                    running;
    glv::Data               data;
    glv::PlotFunction1D*    plotter;
    L3::ConstantTimeIterator< L3::LHLV>* iterator; 

    ~VelocityPlotter()
    {
    }

    void update()
    {
        std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > window ;
        iterator->getWindow( window );

        if ( window.size() > 0 )
        {
            data.resize( glv::Data::DOUBLE, 1, window.size() );

            glv::Indexer i(data.size(1));

            int counter = 0;

            int index = 9;

            while( i() && counter < window.size() ) 
            {
                //std::cout << window.size() << ":" << counter << std::endl;
                double d = window[counter++].second->data[index];
                data.assign( d, i[0], i[1] );
            }

            //plotter->data() = data;
        }
    }

};

} // Visualisers
} // L3

#endif
