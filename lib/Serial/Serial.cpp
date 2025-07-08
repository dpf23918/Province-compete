#include "Serial.h"
#include <stdarg.h>

ESP32Serial SerialEx;

ESP32Serial::ESP32Serial(HardwareSerial& ser, int rxPin, int txPin) 
    : _serial(ser), _rxPin(rxPin), _txPin(txPin) {}

void ESP32Serial::begin(uint32_t baudrate) {
    if(_rxPin != -1 && _txPin != -1) {
        _serial.setPins(_rxPin, _txPin);
    }
    _serial.begin(baudrate);
    
    // 使用Lambda捕获this指针处理接收
    _serial.onReceive([this]() {
        while(this->_serial.available()) {
            this->_rxData = this->_serial.read();
            this->_rxFlag = true;
        }
    });
}

void ESP32Serial::sendByte(uint8_t data) {
    _serial.write(data);
}

void ESP32Serial::sendArray(const uint8_t* array, size_t length) {
    _serial.write(array, length);
}

void ESP32Serial::sendString(const char* str) {
    _serial.print(str);
}

void ESP32Serial::sendNumber(uint32_t number, uint8_t length) {
    char buf[10];
    snprintf(buf, sizeof(buf), "%0*lu", length, number);
    sendString(buf);
}

void ESP32Serial::printf(const char* format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    sendString(buf);
}

bool ESP32Serial::available() {
    return _rxFlag;
}

uint8_t ESP32Serial::readByte() {
    _rxFlag = false;
    return _rxData;
}