#ifndef L3_VISUALISERS_PLOTTERS_H
#define L3_VISUALISERS_PLOTTERS_H

#include "L3.h"

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Controls.h"

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
                init = false;
                
                x_tmp = 0;
                y_tmp = 0;

            }


            virtual ~VelocityPlotter()
            {
            }

            int index;
            bool init; 
            double x_limit, y_limit;
            
            int y_tmp;
            double x_tmp;
            
            boost::weak_ptr< glv::Plot > plot_parent; 
            boost::weak_ptr< L3::ConstantTimeIterator< L3::LHLV> > iterator; 

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
