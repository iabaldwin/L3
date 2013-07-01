#include "Core.h"

namespace L3
{
    void Updater::update()
    {
        L3::ReadLock lock( this->mutex );
        std::for_each( updateables.begin(), updateables.end(), std::mem_fun( &Updateable::update ) );

        //for( std::list < Updateable* >::iterator it=updateables.begin();
                //it != updateables.end();
                //it++ )
        //{
            //std::cout << "Updating: " << std::distance( updateables.begin(), it ) << "(" << typeid( &*it ).name() << ")" << std::endl;
            //(*it)->update();
        //}
        //std::cout << "----------" << std::endl;
        
        lock.unlock();
    }

    void Updater::remove( Updateable* updateable )
    {
        L3::WriteLock lock( this->mutex );

        if( updateable )
            updateables.erase( std::find( updateables.begin(), 
                        updateables.end(), 
                        updateable
                        ) );
        lock.unlock();
    }

}
