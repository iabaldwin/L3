#include <iostream>
#include <numeric>
#include <iterator>
#include <vector>

#include "Definitions.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Utils.h"

namespace iab
{
    template <typename InputIterator, typename OutputIterator, typename BinaryOp >
        OutputIterator adjacent_difference ( InputIterator first, InputIterator last, OutputIterator result, BinaryOp op  )
        {
            typename std::iterator_traits<InputIterator>::value_type val,prev;
            
            *result++ = prev = *first++;
            //while (first!=last) {
                //val = *first++;
                ///[>result++ = binary_op(val,prev);
                //prev = val;
            //}
            
            return result;
        }
}

//struct differentiator : std::binary_function< std::pair< double, boost::shared_ptr< L3::LMS151 > >, std::pair< double, boost::shared_ptr< L3::LMS151 > >, double >
struct differentiator 
{

template <typename T>
    double operator()( T current, T previous )
    {
        return current.first - previous.first;
    }

};

std::ostream& operator<<( std::ostream& o, const std::vector< double >& data )
{

    std::copy( data.begin(), data.end(), std::ostream_iterator<double>( o, "\n" ) );

    return o;
}

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/");
    if ( !( dataset.validate() && dataset.load() ) )
        exit(-1);

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::LMS151 > iterator( dataset.LIDAR_readers.begin()->second );
  
    iterator.swathe_length = 30.0;

    double time = dataset.start_time;
        
    std::cout.precision(15);

    // Run
    while (true)
    {
        if ( !iterator.update( time += 1 ) )
            exit(-1);

        std::vector<double> differences( iterator.window.size() );
        std::vector<double>::iterator differences_ptr = differences.begin();

        for( std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > >::iterator it = (iterator.window.begin()+1);
                it != iterator.window.end();
                it++ )
            *differences_ptr++ = (*it).first - (*(it-1)).first;

        std::cout << differences << std::endl;

        std::cout << time - iterator.window.back().first << std::endl;

        std::cout << time << "-->" << iterator.window.front().first << "\t:" << iterator.window.back().first << "\t:" << iterator.window.back().first - iterator.window.front().first <<  " \t(" << iterator.window.size() << ")" << std::endl;
        
        usleep( .1*1e6 );
    } 
}
