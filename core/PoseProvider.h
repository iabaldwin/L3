#ifndef L3_POSE_PROVIDER_H
#define L3_POSE_PROVIDER_H

struct PoseProvider : std::unary_function< L3::SE3, void >
{
    virtual L3::SE3 operator()() = 0;
};

struct CircularPoseProvider : PoseProvider
{
    CircularPoseProvider() : counter(0), w(0.1)
    {
    }

    ~CircularPoseProvider()
    {
    }

    int             counter;
    double          w;
        
    L3::SE3 operator()() 
    {
        
    }
};

#endif
