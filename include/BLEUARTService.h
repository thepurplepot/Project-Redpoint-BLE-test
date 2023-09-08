#pragma once
#ifndef _BLEUARTSERVICE_H
#define _BLEUARTSERVICE_H

#include "BLEServiceHandler.h"

static const char NF_UART_SERVICE_UUID[] =
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char NF_UART_RX_CHAR_UUID[] =
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char NF_UART_TX_CHAR_UUID[] =
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// If true, send the timestamp every UART_TX_HEARTBEAT_DELAY ticks
static const bool SEND_UART_TX_HEARTBEAT = false;
static const int UART_TX_HEARTBEAT_DELAY = (1000 - 10) / 10;

class UARTRxCallbacks : public BLECharacteristicCallbacks {
 public:
  UARTRxCallbacks(BLECharacteristic *txChar);

 private:
  void onWrite(BLECharacteristic *ch);

  BLECharacteristic *txChar_;
};

class BLEUARTServiceHandler : public BLEServiceHandler {
 public:
  BLEUARTServiceHandler(BLEServiceManager &manager);

  void tick();

 private:
  BLECharacteristic *txChar_;
  BLECharacteristic *rxChar_;
  unsigned long nextTxTimeMs_ = 0;
};

#endif  // _BLEUARTSERVICE_H
