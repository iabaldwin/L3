#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Datatypes.h"
#include "Utils.h"
#include "Reader.h"
#include "Writer.h"
#include "Components.h"

int main (int argc, char ** argv){

    glv::GLV top;
    glv::Window win(1200, 600, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Pose sequence
    std::vector<L3::Pose*> poses;
    std::auto_ptr<L3::IO::PoseReader> reader( new L3::IO::PoseReader() );
    
    reader->open( "/Users/ian/code/python/tools/test/OxTS.ins" );
    reader->read();

    if ( !reader->extract( poses ) )
        throw std::exception();

    //L3::UTILS::localisePoseChain( poses, L3::UTILS::BEGBROKE() );
    L3::UTILS::localisePoseChainToOrigin( poses );
    
    // DBG
    //L3::IO::Writer w; 
    //if ( w.open( "test.txt" ) )
        //w << poses;

    L3::Visual::PoseChain chain(poses);
    top << chain;
 
        
    win.setGLV(top);
    glv::Application::run();
}


