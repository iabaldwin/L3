#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"

enum ColorTYPE
{
    RED,
    GREEN,
    BLUE
};

struct ColorCycler
{
    ColorCycler() : counter(0)
    {
        
    }

    int counter;

    ColorTYPE operator()()
    {
        return (ColorTYPE)(counter++%3);
    }


};

struct Section
{

    Section( unsigned int SIZE, L3::Point<double>* ptr, ColorTYPE type ) : size(SIZE)
    {
        vertices = new glv::Point3[size];
        colors = new glv::Color[size];
 
        for( unsigned int counter=0; counter<size; counter++ )
        {
            vertices[counter]( ptr[counter].x, ptr[counter].y, ptr[counter].z );
       
            switch (type)
            {
                case RED:
                    colors[counter].set( 255, 0, 0 );
                    break;
                
                case GREEN:
                    colors[counter].set( 0, 255, 0 );
                    break;
                
                case BLUE:
                    colors[counter].set( 0, 0, 255 );
                    break;

                default:
                    std::cerr << "Error..." << std::endl;
                    break;
            };
        }
    
    }

    glv::Point3*    vertices;
    glv::Color*     colors; 
    unsigned int    size;

    ~Section()
    {
        delete [] vertices;
        delete [] colors;
    }

};

struct ColoredExperienceRenderer : L3::Visualisers::Leaf
{

    ColoredExperienceRenderer( boost::shared_ptr<L3::Experience> EXPERIENCE ) : experience(EXPERIENCE)
    {
        experience->running = false;

        std::cout << "Synching..." << std::endl;
        //usleep( 2*1e6 );
        
        int section_counter = 0;
     
        ColorCycler cycler;

        while( true )
        {
            try
            {
                // Load the experience
                std::pair< long unsigned int, L3::Point<double>* > load_result = experience->load( section_counter );
            
#ifndef NDEBUG
                std::cout << load_result.first<< ":" << load_result.second << std::endl;
#endif

                sections.push_back( new Section( load_result.first, load_result.second, cycler() ) );
            }
            catch( ... )
            {
                break;
            }
            
            section_counter++;

        }

    }

    ~ColoredExperienceRenderer()
    {
        for( std::vector< Section* >::iterator it = sections.begin(); 
                it != sections.end();
                it++ )
            
            delete (*it);

    }

    void onDraw3D( glv::GLV& g )
    {
        for( std::vector< Section* >::iterator it = sections.begin(); 
                it != sections.end();
                it++ )
        {
            glv::draw::paint( glv::draw::Points, (*it)->vertices, (*it)->colors, (*it)->size );
        }
    }

    std::vector< Section* > sections;

    boost::shared_ptr<L3::Experience> experience;

};


int main (int argc, char ** argv)
{
    /*
     *L3
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    L3::ExperienceLoader experience_loader( dataset );

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::ExperienceRenderer");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Point cloud renderer
    L3::Visualisers::Composite              composite;
    L3::Visualisers::Controller*            controller = new L3::Visualisers::BasicPanController();
    L3::Visualisers::Grid                   grid;
    ColoredExperienceRenderer               colored_experience_renderer( experience );
    
    // Associate controller
    composite.addController( controller ).stretch(1,1);

    // Combine
    top << (composite << grid << colored_experience_renderer ) ;

    // Run
    win.setGLV(top);
    win.fit(); 
    glv::Application::run();

}
