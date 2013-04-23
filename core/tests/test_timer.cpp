#include <boost/chrono.hpp>
#include <cmath>

int main()
{
    boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();

    while (true)
    {
        usleep( .5*1e6 );
        boost::chrono::duration<double> sec = boost::chrono::system_clock::now() - start;
        std::cout << "took " << sec.count() << " seconds\n";
    }
}
