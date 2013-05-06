#include "ScanMatching.h"

#include <ICP/icp.h>
#include <ICP/icpPointToPlane.h>

int do_projection( const L3::LMS151& current_scan, double* matrix, double threshold = 5.0 )
{

    double* matrix_ptr = matrix;

    float range,angle,x,y,z=0;
  
    int counter = 0;

    for (int scan_counter=0; scan_counter<541; scan_counter++) 
    {
        // Compute angle 
        angle = scan_counter*current_scan.angle_spacing +  current_scan.angle_start; 
        range = current_scan.ranges[scan_counter];  

        if ( range < threshold )
            continue;

        x = range*cos( angle );
        *matrix_ptr++ = x; 
        y = range*sin( angle );
        *matrix_ptr++ = y; 
        *matrix_ptr++ = z; 
   
        counter++;
    }

    return counter;
}

namespace L3
{
namespace ScanMatching
{
    bool ICP::match(  const L3::LMS151& current_scan )
    {
        if ( !initialised )
        {
            scan.reset( new double[541*3] );
            scan_points = do_projection( current_scan, scan.get() );

            bool retval = initialised; 
            initialised = true;
            return retval;
        }

        putative.reset( new double[541*3] );
            
        putative_points  = do_projection( current_scan, putative.get() );
        
        Matrix R = Matrix::eye(3);
        Matrix t(3,1);
        
        IcpPointToPlane icp( scan.get(), scan_points, 3);
        icp.fit(putative.get(),putative_points,R,t,-1);

        // Do swap
        scan.swap( putative );
        scan_points = putative_points;
    }
    
    bool Engine::update( double time )
    {
        std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;
        this->windower->getWindow( window );

        L3::Timing::ChronoTimer t;
        if ( window.size() > 0 )
            matcher->match( *(window.front().second ) ); 
    
    }


}
}
