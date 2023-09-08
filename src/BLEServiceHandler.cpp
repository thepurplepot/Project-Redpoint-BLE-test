#include "BLEServiceHandler.h"
#include "BLEServiceManager.h"

BLEServiceHandler::BLEServiceHandler(BLEServiceManager &manager,
                                     const char uuid[])
    : uuid(uuid), bleService_(manager.bleServer.createService(uuid)) {
  manager.addServiceHandler(*this);
}