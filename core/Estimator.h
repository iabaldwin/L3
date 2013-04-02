#ifndef L3_ESTIMATOR_H
#define L3_ESTIMATOR_H

typedef gsl_histogram2d 2D_HIST;

namespace L3
{
namespace Estimator
{

/*
 *  Base estimator
 */
struct Estimator
{
    bool estimate (  2D_HIST* exp, 2D_HIST* swathe, L3::SE3& estimated_pose ) = 0;

    virtual ~Estimator()
    {

    }


};

struct KLEstimator : Estimator
{
    bool estimate (  2D_HIST* exp, 2D_HIST* swathe, L3::SE3& estimated_pose ) 
    {

        return true;
    }

};


}
}

#endif
