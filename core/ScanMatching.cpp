#include "ScanMatching.h"

#include <boost/scoped_array.hpp>

#include <ICP/icp.h>
#include <ICP/icpPointToPlane.h>


namespace L3
{
namespace ScanMatching
{
    bool ICP::match(  const L3::LMS151& scan )
    {
        if ( !initialised )
        {
            this->scan = scan;
           
            initialised = true;

            return false;
        }

        boost::scoped_array<double> M( new double[541*3] );

        for( int i=0; i<541; i++ )
        {
            M[i*3+0] = rand()%100;
            M[i*3+1] = rand()%100;
            M[i*3+2] = rand()%100;
        }
 
        boost::scoped_array<double> T( new double[541*3] );

        for( int i=0; i<541; i++ )
        {
            T[i*3+0] = rand()%100;
            T[i*3+1] = rand()%100;
            T[i*3+2] = rand()%100;
        }
    
        Matrix R = Matrix::eye(3);
        Matrix t(3,1);

        //std::cout << std::endl << "Running ICP (point-to-plane, no outliers)" << std::endl;
        //IcpPointToPlane icp(M,541,3);
        IcpPointToPlane icp(M.get(),541,3);
        icp.fit(T.get(),541,R,t,-1);

        // results
        //std::cout << std::endl << "Transformation results:" << std::endl;
        //std::cout << "R:" << std::endl << R << std::endl << std::endl;
        //std::cout << "t:" << std::endl << t << std::endl << std::endl;
    
    }
    
    bool Engine::update( double time )
    {
        std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;
        this->windower->getWindow( window );

        //TODO:
        //Do we get the closest scan at time, or the next in the sequence?

        //1. Get scan
        //2. Add it to the matcher

        L3::Timing::ChronoTimer t;
        if ( window.size() > 0 )
        {
            //t.begin();
            //matcher->match( *(window.front().second ) ); 
            //std::cout << t.elapsed() << std::endl;
        }
    }


}
}
