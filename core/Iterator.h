#include <iostream>
#include <fstream>

namespace L3
{

class Iterator
{

    public:
    
        Iterator( L3::Dataset* DATASET ) : dataset(DATASET)
        {
        }

    private:

        L3::Dataset* dataset;


};

class ConstantTime : public Iterator
{

    ConstantTime( L3::Dataset* DATASET ) : Iterator( DATASET )
    {
    }

}



} // L3
