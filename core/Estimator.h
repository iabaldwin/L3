#ifndef L3_ESTIMATOR_H
#define L3_ESTIMATOR_H

namespace L3
{
namespace Estimator
{

struct Stepper
{



};



/*
 *  Base estimator
 */
struct Estimator
{
    bool estimate ( 2D_HIST* exp, 2D_HIST* swathe, L3::SE3& estimated_pose, L3::SE3 pose_guess ) = 0;

    virtual ~Estimator()
    {

    }


};

struct KLEstimator : Estimator
{
    bool estimate ( 2D_HIST* exp, 2D_HIST* swathe, L3::SE3& estimated_pose, L3::SE3 pose_guess ) = 0;
    {
        return true;
    }

};


}
}

#endif
