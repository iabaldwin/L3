#include "Windower.h"
#include "Dataset.h"
#include "Iterator.h"
#include "Datatypes.h"
#include "Definitions.h"
#include "PoseWindower.h"

int main()
{
    std::cout.precision( 16 );
    
    /*
     *Dataset 
     */
    boost::shared_ptr< L3::Dataset > dataset(  new L3::Dataset( "/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/" ) );
    dataset->validate(); 
    dataset->load();
    
    boost::shared_ptr< L3::ConstantTimeIterator< L3::SE3 > > pose_iterator;
    pose_iterator = boost::make_shared< L3::ConstantTimeIterator<L3::SE3> >( dataset->pose_reader );

    double time = dataset->start_time;

    /*
     *Manual
     */
    //boost::shared_ptr< L3::SlidingWindow<L3::SE3> >  pose_reader;
    //pose_reader = L3::WindowerFactory<L3::SE3>::constantTimeWindow( "/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/L3/OxTS.ins", 30 ) ;
    //pose_reader->initialise(); 
    
    //Poco::Thread* thread = new Poco::Thread();
    //thread->start( *pose_reader );

    //boost::shared_ptr< L3::ConstantTimeIterator< L3::SE3 > > pose_iterator;
    //pose_iterator = boost::make_shared< L3::ConstantTimeIterator<L3::SE3> >( pose_reader );
   
    //double time = 1334608441.524997;
  
    //boost::shared_ptr< L3::ConstantTimeWindower< L3::SE3 > >    oracle;
    //oracle = boost::make_shared< L3::ConstantTimeWindower< L3::SE3 > > ( pose_iterator.get() );

    while (true)
    {
        pose_iterator->update( time );

        std::cout << pose_iterator->window.size() << std::endl;

        time +=.2;
        std::cout << time << std::endl;

        //std::cout << oracle->operator()() << std::endl;

        usleep( .2*1e06) ;
    } 
    
}

