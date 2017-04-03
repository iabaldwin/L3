#ifndef L3_VISUALISERS_HANDLER_H
#define L3_VISUALISERS_HANDLER_H

namespace L3
{
namespace Visualisers
{

  struct Handler
  {
    virtual bool onEvent( glv::Event::t e, glv::GLV& g) = 0;
  };


  struct Playback : Handler
  {
    Playback( L3::DatasetRunner* runner ) : runner(runner)
    {
    }

    L3::DatasetRunner* runner;

    bool onEvent( glv::Event::t e, glv::GLV& g) 
    {
      // Dump history
      if ( e == glv::Event::KeyDown) 
      {
        const glv::Keyboard& k = g.keyboard();

        // If we just use space, have to make sure that
        // the scripting interface is not visible, otherwise
        // we can't type spaces without pausing the engine

        //if ( k.key() 
      }

    }

  };

}
}


#endif

