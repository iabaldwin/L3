#pragma once

namespace L3
{
  namespace Visualisers
  {
    struct DatasetController : glv::EventHandler
    {
      L3::DatasetRunner* runner;

      bool onEvent( glv::View& v, glv::GLV& g)
      {
        const glv::Keyboard& k = g.keyboard();

        switch (k.key())
        {
          case 32: // Space-bar ASCII
            runner->paused = !runner->paused;
            break;
          default:
            break;
        }
        return true;
      }
    };

    struct ReflectanceController : glv::EventHandler
    {

      ReflectanceController( bool& switch_val ) : switch_val(switch_val)
      {
      }

      bool& switch_val;

      bool onEvent( glv::View& v, glv::GLV& g)
      {
        const glv::Keyboard& k = g.keyboard();

        switch (k.key())
        {
          case 32:
            switch_val = !switch_val;
            break;
          default:
            break;
        }
        return true;
      }
    };
  }
}
