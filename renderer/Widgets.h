#ifndef L3_VISUALISERS_WIDGETS_H
#define L3_VISUALISERS_WIDGETS_H

#include <GLV/glv.h>

#include "L3.h"

#include <deque>
#include <boost/shared_ptr.hpp>

namespace L3
{
namespace Visualisers
{

    template <typename T>
        struct ResettableSlider : glv::View
    {
        ResettableSlider( const std::string& label_text, T lower=1, T upper=10 )  
            : glv::View( glv::Rect(150,20) ),
            reset( false )
        {

            table = boost::make_shared< glv::Table >( "x x x,", 5, 0 );

            slider = boost::make_shared< glv::Slider >();
    
            slider->interval( upper, lower );

            label = boost::make_shared< glv::Label >( label_text );
            label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR );
            
            //reset_toggle = boost::make_shared< glv::Button >( glv::Rect(20), false );
            reset_toggle = boost::make_shared< glv::Button >( glv::Rect(20), true );

            *table <<  *slider;
            *table <<  *reset_toggle;
            *table <<  *label;
  
            table->arrange();
            table->fit();

            *this << *table;
       
            this->fit();
       
            this->disable( glv::DrawBorder );
        }

        T initial_val;

        bool reset;

        boost::shared_ptr< glv::Table > table;
        boost::shared_ptr< glv::Label > label;
        boost::shared_ptr< glv::Widget > slider;
        boost::shared_ptr< glv::Button > reset_toggle;

        void onDraw( glv::GLV& g )
        {
            if ( reset_toggle->getValue() )
                slider->setValue( initial_val ); 
        }

        void attach( T& t )
        {
            this->initial_val = t;
            slider->attachVariable( t ); 
        }

    };



    struct DensityController : glv::View
    {
        DensityController() : glv::View( glv::Rect(180,20 ) )
        {
            this->disable( glv::DrawBorder );
        }

        std::deque< boost::shared_ptr< glv::Label > > labels;

        std::deque< boost::shared_ptr< glv::Widget > > widgets;

        void attachHistogram( boost::shared_ptr< L3::Histogram<double> > histogram );

    };

    struct FundamentalControls : glv::Table
    {
        FundamentalControls();

        boost::shared_ptr< glv::Table > _table;

        std::deque< boost::shared_ptr< glv::View> > views;
        std::deque< boost::shared_ptr< glv::Label > > labels;
        std::deque< boost::shared_ptr< glv::Widget > > widgets;

        glv::Label widget_label;

        void associateVelocitySource( boost::shared_ptr< FilteredScanMatchingVelocityProvider> ptr);

    };

    struct PointCloudControls : glv::Table
    {

        PointCloudControls();

        std::deque< boost::shared_ptr< glv::View> > views;
        std::deque< boost::shared_ptr< glv::Label > > labels;
        std::deque< boost::shared_ptr< glv::Widget > > widgets;

    
        void attach( float& a, float& b, float& c );

        float a, b, c;

    };

}
}

#endif

