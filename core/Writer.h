#ifndef L3_WRITER_H
#define L3_WRITER_H

#include <vector>
#include <iterator>
#include <fstream>
#include "Datatypes.h"


namespace L3
{
namespace IO
{

class Writer 
{

    public:
        virtual ~Writer();
    
        bool open( const std::string& f );
        

    protected:

        std::ofstream stream;


    private:
        friend  L3::IO::Writer& operator<<( L3::IO::Writer& o, const std::vector<Pose*>& poses );

};



}
}

#endif
