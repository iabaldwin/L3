#ifndef L3_VISUALISERS_PLOTTERS_H
#define L3_VISUALISERS_PLOTTERS_H

#include "L3.h"

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Controls.h"
#include "RenderingUtils.h"

namespace L3
{
    namespace Visualisers
    {
        struct VelocityPlotter : glv::Plottable, Lockable, Updateable
        {
            VelocityPlotter( bool filtered )
                : glv::Plottable( glv::draw::LineStrip, 1 ),
                    filtered(filtered)
            {
            }


            virtual ~VelocityPlotter()
            {
            }


            int index;
            bool filtered; 
            double x_limit, y_centroid;
            boost::shared_ptr< variable_lock<double> > time_lock;

            boost::weak_ptr< glv::Plot > plot_parent; 
            boost::weak_ptr< L3::VelocityProvider > iterator; 

            void setTime( double& time )
            {
                time_lock = boost::make_shared< variable_lock<double> >( boost::ref(time) ); 
            }

            void onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i)
            {
                L3::ReadLock reader( this->mutex );
                int counter = 0; 
                while(i()){
                    double x = d.at<double>(0, counter );
                    double y = d.at<double>(1, counter );
                    counter++;
                    
                    g.addVertex(x, y);
                }
                reader.unlock();
            }

            void update();
        };

        struct LinearVelocityPlotter : VelocityPlotter
        {
            LinearVelocityPlotter( bool filtered = true)  : VelocityPlotter(filtered)
            {
                index = 0;
                this->color( glv::Color( 1,0,0 ) );
            }
        };

        struct RotationalVelocityPlotter : VelocityPlotter
        {
            RotationalVelocityPlotter( bool filtered = true )  : VelocityPlotter(filtered)
            {
                index = 3;
                this->color( glv::Color( 0,1,0 ) );
            }
        };

    }   // Visualisers
}       // L3

#endif
