#ifndef L3_VISUALISERS_DATASET_CONTROLLER_H
#define L3_VISUALISERS_DATASET_CONTROLLER_H

namespace L3
{
namespace Visualisers
{
    struct DatasetController : glv::EventHandler
    {

        DatasetController() 
        {
        }

        L3::DatasetRunner* runner;

        bool onEvent( glv::View& v, glv::GLV& g)
        {
            const glv::Keyboard& k = g.keyboard();

            switch (k.key())
            {
                case ' ':
                    runner->paused = !runner->paused;

            }
        }

    };

}
}




#endif

