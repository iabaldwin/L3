#include "Runner.h"

namespace L3
{

    bool EstimatorRunner::update( double time )
    {

#ifndef NDEBUG
        boost::timer t;
#endif
        swathe_builder->update( time );

        boost::shared_ptr<L3::PointCloud<double> > experience_cloud;

        L3::SE3 pose = (*provider)();

        experience->update( pose.x, pose.y );
        experience->getExperienceCloud( experience_cloud );

#ifndef NDEBUG
        std::cout << "Experience\t" << t.elapsed() << std::endl;
#endif
        projector->project( swathe_builder->swathe );

#ifndef NDEBUG
        std::cout << "Projection\t" << t.elapsed() << std::endl;
#endif

        (*estimator)( &*experience_cloud, projector->cloud, L3::SE3::ZERO() );

#ifndef NDEBUG
        std::cout << "Estimation\t" << t.elapsed() << std::endl;
#endif

    }
}

