#pragma once
#ifndef _BLESERVICEHANDLER_H
#define _BLESERVICEHANDLER_H

#include <BLEServer.h>
#include <BLEService.h>

class BLEServiceManager;

class BLEServiceHandler {
 public:
  std::string uuid;
  BLEServiceHandler(BLEServiceManager &manager, const char uuid[]);

  virtual void start() { bleService_->start(); }

  virtual void tick() {}

 protected:
  BLEService *bleService_;
};
#endif // _BLESERVICEHANDLER_H
