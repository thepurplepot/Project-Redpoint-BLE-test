#include "BLEUARTService.h"

#include <Arduino.h>
#include <BLE2902.h>

UARTRxCallbacks::UARTRxCallbacks(BLECharacteristic *txChar) : txChar_(txChar) {};

void UARTRxCallbacks::onWrite(BLECharacteristic *ch) {
    std::string value = ch->getValue();
    if (value.length() > 0) {
        Serial.print("Rx: ");
        Serial.write(value.c_str());
        if (value[value.length() - 1] != '\n') {
            Serial.println();
        }
        if (value == "ping" || value == "ping\n") {
            Serial.write("Tx: pong\n");
            static uint8_t data[] = "pong\n";
            txChar_->setValue(data, sizeof data - 1);
            txChar_->notify();
        }
    }
}


BLEUARTServiceHandler::BLEUARTServiceHandler(BLEServiceManager &manager)
    : BLEServiceHandler(manager, NF_UART_SERVICE_UUID) {
    txChar_ = bleService_->createCharacteristic(
        NF_UART_TX_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    txChar_->addDescriptor(new BLE2902());

    rxChar_ = bleService_->createCharacteristic(
        NF_UART_RX_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    rxChar_->setCallbacks(new UARTRxCallbacks(txChar_));
}

void BLEUARTServiceHandler::tick() {
    unsigned long now = millis();
    if (SEND_UART_TX_HEARTBEAT && now > nextTxTimeMs_) {
        static char buffer[10];
        int len = snprintf(buffer, sizeof buffer, "%ld\n", now);
        if (0 <= len && len <= sizeof buffer) {
            txChar_->setValue((uint8_t *)buffer, len);
            txChar_->notify();
        } else {
            Serial.println("Tx buffer overflow");
        }
        nextTxTimeMs_ = now + UART_TX_HEARTBEAT_DELAY;
    }
}