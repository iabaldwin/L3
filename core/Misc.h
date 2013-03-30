#ifndef L3_MISC_H
#define L3_MISC_H

namespace L3
{
namespace Misc
{

/*
 *Configuration
 */
    
std::list <std::string> getDatasetConfigurations()
{
    boost::filesystem::directory_iterator itr( boost::filesystem::path( "/Users/ian/code/datasets/configuration/missions/" ) );
    std::list <std::string> datasets;

    while( itr != boost::filesystem::directory_iterator() )
    {
        if ( isdigit( itr->path().leaf().string()[0] ))
        {
            datasets.push_front( itr->path().string() );
        }

        itr++;
    }

    return datasets;

}

}
}
#endif

