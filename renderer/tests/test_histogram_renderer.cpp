#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

struct histogram_renderer : L3::Visualisers::Component
{

    histogram_renderer()
    {
        num_bins = 50;
        histogram_vertices = new glv::Point3[4*num_bins*num_bins];
        histogram_colors = new glv::Color[4*num_bins*num_bins];
    }

    size_t num_bins;
    glv::Point3* histogram_vertices;
    glv::Color*  histogram_colors;

    void onDraw3D(glv::GLV& g)
    {
        far(1000);

        L3::PointCloud<double> cloud;
        cloud.points = new L3::Point<double>[10000];
        cloud.num_points = 10000;
      
        L3::PointCloud<double>::ITERATOR it = cloud.begin();
        while( it != cloud.end() )
        {
            *it++ = L3::Point<double>( random()%100, random()%100, random()%100 );
        }

        L3::histogram<double> hist(50,50,num_bins);

        hist( &cloud );

        float delta = hist.delta;
        int counter = 0; 

        glv::Point3 quad_vertices[4];
        glv::Color quad_colors[4];

        glv::draw::translateX( -50 );
        glv::draw::translateY( -50 );
        glv::draw::translateZ( -250 );
        
        for( unsigned int i=0; i < hist.num_bins; i++ )
        {
            for( unsigned int j=0; j < hist.num_bins; j++ )
            {
                unsigned int val = hist.bin( i, j );
          
                glv::Color c = glv::Color( val/10.0 );

                std::pair<float,float> coords = hist.coords( i, j);

                quad_colors[0] = c;
                quad_vertices[0]( coords.first-delta/2.5, coords.second-delta/2.5, val );
                quad_colors[1] = c;
                quad_vertices[1]( coords.first-delta/2.5, coords.second+delta/2.5, val );
                quad_colors[2] = c;
                quad_vertices[2]( coords.first+delta/2.5, coords.second+delta/2.5, val );
                quad_colors[3] = c;
                quad_vertices[3]( coords.first+delta/2.5, coords.second-delta/2.5, val );
        
                glv::draw::paint( glv::draw::TriangleFan, quad_vertices, quad_colors, 4 );
            }
        }
    }
};

int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    histogram_renderer t;
    top << t;

    win.setGLV(top);
    glv::Application::run();
}


