#include "Core.h"

namespace L3
{

  void Updater::update() {
    L3::ReadLock lock(this->mutex);
    std::for_each(updateables.begin(), updateables.end(), std::mem_fun(&Updateable::update));
    lock.unlock();
  }

  void Updater::remove(Updateable* updateable) {
    CHECK_NOTNULL(updateable);
    L3::WriteLock lock(this->mutex);
    updateables.erase(std::find(updateables.begin(),
          updateables.end(),
          updateable));
    lock.unlock();
  }

} // L3
