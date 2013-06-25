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

        //boost::weak_ptr< L3::DatasetRunner > runner;
        L3::DatasetRunner* runner;

        bool onEvent( glv::View& v, glv::GLV& g)
        {
            //boost::shared_ptr< L3::DatasetRunner > runner_ptr = runner.lock();
            //if( !runner_ptr )
                //return false;

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

