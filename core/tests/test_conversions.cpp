#include "L3.h"

int main()
{
    for( int i=0; i<100; i++ )
    {
        //L3::SE3 pose( 0,0,0, M_PI/2, M_PI/4, M_PI/8 );
        L3::SE3 pose( 0,0,0, (rand()%10/10.0)*M_PI/2, (rand()%10/10.0)* M_PI/4, (rand()%10/10.0)*M_PI/8 );

        std::cout << pose << std::endl;
        std::cout << L3::Utils::Math::poseFromRotation( pose.getHomogeneous() ) << std::endl;

        std::cout << "-------------------" << std::endl;
    }
}
