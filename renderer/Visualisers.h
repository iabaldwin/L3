#ifndef L3_VISUAL_COMPONENTS_H
#define L3_VISUAL_COMPONENTS_H

#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"

namespace L3
{
namespace Visualisers
{

struct Component : glv::View3D{

    Component()
    {
        /*
         *If you don't have an active
         *    view, then the onDraw
         *    methods are not called. 
         *    However, this stretches
         *    to the full size of the
         *    element.
         */
    }


    virtual void onDraw3D(glv::GLV& g)
    {
    }
    virtual void onDraw2D(glv::GLV& g)
    {
    }

};

struct Leaf 
{
    virtual void onDraw3D( glv::GLV& g )
    {}

    virtual void onDraw2D( glv::GLV& g )
    {}

    double time;
};

/*
 * Composite renderer
 *  
 *  Render multiple leaf viewers 
 *
 */
struct Composite : glv::View3D 
{

    Composite() : time(0.0)
    {
        stretch(1,1); 
        
        far( 300 );
    }

    double time;

    virtual void onDraw3D(glv::GLV& g)
    {
        std::list<Leaf*>::iterator it = components.begin();
        glv::draw::translateZ( -250 );

        std::cout << time << std::endl;

        time += .1;
        while( it != components.end() )
        {
            (*it)->time = this->time;
            (*it)->onDraw3D( g );
            it++;
        }
    
    }

    virtual void onDraw2D( glv::GLV& g)
    {
    }

    Composite& operator<<( Leaf& leaf )
    {
        components.push_back( &leaf );
        return *this;
    }

    std::list<Leaf*> components; 

};

/*
 *  Pose chain renderer
 *
 *      Render a static chain of poses
 *
 */
struct PoseChainRenderer : Leaf
{
    glv::Color* colors;
    glv::Point3* vertices;

    PoseChainRenderer( std::vector< std::pair< double, boost::shared_ptr<L3::Pose> > >& POSES ) : poses(POSES)
    {
        colors = new glv::Color[poses.size()];
        vertices = new glv::Point3[poses.size()];

        int counter = 0;
        for( POSE_SEQUENCE_ITERATOR it=poses.begin(); it < poses.end(); it++ )
        {
            vertices[counter]( it->second->x, it->second->y, 0 );
            colors[counter] = glv::HSV(0.6, .1, 0.45+0.55);
            counter++;
        }

    }

    ~PoseChainRenderer()
    {
        delete [] colors;
        delete [] vertices;
    }

   
    void onDraw3D(glv::GLV& g)
    {
        glv::draw::paint( glv::draw::Points, vertices, colors, poses.size() );
    }

    void onDraw2D(glv::GLV& g)
    {
    }

    std::vector< std::pair< double, boost::shared_ptr<L3::Pose> > >& poses;
};

/*
 *Cloud Renderer
 */
template <typename T>
struct CloudRenderer : Leaf
{
    CloudRenderer( L3::PointCloud<T>* CLOUD ) : cloud(CLOUD)
    {
        colors   = new glv::Color[cloud->size()];
        vertices = new glv::Point3[cloud->size()];
   
        // Build the cloud
        typename std::vector< Point<T> >::iterator it;
        it = cloud->data.begin();

        int counter = 0;
        while( it != cloud->data.end() )
        {
            vertices[counter]( (*it).x, (*it).y, (*it).z); 
            colors[counter] = glv::HSV(0.6, .1, (*it).z*0.45+0.55);
            it++; counter++;
        }
    
    };

    ~CloudRenderer()
    {
        delete [] colors;
        delete [] vertices;
    }

    void onDraw3D( glv::GLV& g )
    {
        L3::SE3 pose(1,0,0,0,0,0);
        cloud->transform( &pose );

        typename std::vector< Point<T> >::iterator it;
        it = cloud->data.begin();

        int counter = 0;
        while( it != cloud->data.end() )
        {
            vertices[counter]( (*it).x, (*it).y, (*it).z); 
            colors[counter] = glv::HSV(0.6, .1, (*it).z*0.45+0.55);
            it++; counter++;
        }
    
        glv::draw::paint( glv::draw::Points, vertices, colors, cloud->size());

    }
    
    glv::Color*         colors;
    glv::Point3*        vertices;
    L3::PointCloud<T>*  cloud;

};

/*
 * Iterator renderer
 *
 *  Render the poses from a dataset iterator
 *
 */
template <typename T>
struct IteratorRenderer : Leaf
{

    IteratorRenderer( L3::Iterator<T>* ITERATOR )  : iterator(ITERATOR )
    {
        L3::Utils::BEGBROKE b;
   
        x = b.x + 100;
        y = b.y + 100;
    }

    double x,y;

    void onDraw3D( glv::GLV& g )
    {
        // Update the iterator
        iterator->update( time );

        // Reserve
        glv::Color* colors = new glv::Color[iterator->window.size()];
        glv::Point3* vertices = new glv::Point3[iterator->window.size()];;

        typename L3::Iterator<T>::WINDOW_ITERATOR it = iterator->window.begin();

        int counter = 0;
        while( it != iterator->window.end() )
        {
            vertices[counter]( it->second->x - this->x , it->second->y - this->y, 0 );
            it++; 
            counter++;
        }
    
        glv::draw::paint( glv::draw::Points, vertices, colors, counter );

        delete [] colors;
        delete [] vertices;
    }

    L3::Iterator<T>* iterator;

};


/*
 * Swathe renderer
 *
 *  Render the poses from a swathe generator
 *
 */
struct SwatheRenderer : Leaf
{
    SwatheRenderer( L3::SwatheBuilder* SWATHE_BUILDER )  : swathe_builder(SWATHE_BUILDER)
    {
        L3::Utils::BEGBROKE b;
        x = b.x + 100;
        y = b.y + 100;
        
        // Projector  
        projector.reset( new L3::Projector<double>() );
    }

    std::auto_ptr<L3::Projector<double> > projector;
    double x,y;

    L3::Tools::Timer t;
    void onDraw3D( glv::GLV& g )
    {
        t.begin();
        // Update the swathe_builder
        if ( !swathe_builder->update( time ))
            throw std::exception();

        ////L3::PointCloudXYZ<double> cloud = projector->project( swathe_builder->swathe );
        ////L3::PointCloudXYZ<double> sampled_cloud = L3::samplePointCloud( cloud, 10000 );
        
        //// Reserve
        //glv::Color* colors    = new glv::Color[swathe_builder->swathe.size()];
        //glv::Point3* vertices = new glv::Point3[swathe_builder->swathe.size()];;

        //SWATHE_ITERATOR it = swathe_builder->swathe.begin();

        //int counter = 0;
        //while( it != swathe_builder->swathe.end() )
        //{
            //vertices[counter]( it->first->x - this->x , it->first->y - this->y, 0 );
            //it++; 
            //counter++;
        //}

        //glv::draw::paint( glv::draw::Points, vertices, colors, counter );
        std::cout << t.end() << std::endl;

        //delete [] colors;
        //delete [] vertices;
    }

    L3::SwatheBuilder* swathe_builder;

};



}
}

#endif
