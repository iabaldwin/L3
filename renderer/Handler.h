#pragma once
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
    }
  };
} // Visualisers
} // L3
