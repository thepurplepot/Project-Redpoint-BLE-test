#pragma once
#ifndef _BLESERVICEMANAGER_H
#define _BLESERVICEMANAGER_H

#include <BLEDevice.h>
#include <BLEServer.h>

#include <vector>

#include "BLEServiceHandler.h"

class BLEServiceManager : public BLEServerCallbacks {
  public:
    /* Public Members */
    BLEServer &bleServer;
    /* Constructor */
    BLEServiceManager();
    /* Public Methods */
    uint32_t getConnectedCount();
    void addServiceHandler(BLEServiceHandler &handler);
    void start();
    void tick();

  private:
    /* Private Members */
    std::vector<BLEServiceHandler *> serviceHandlers_;
    BLEAdvertising *adv_;
    bool hasBeenConnected_ = false;
    /* Private Methods */
    void onConnect(BLEServer *server);
    void onDisconnect(BLEServer *server);
};
#endif // _BLESERVICEMANAGER_H
