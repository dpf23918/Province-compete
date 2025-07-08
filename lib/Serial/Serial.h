#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>
#include <HardwareSerial.h>

class ESP32Serial {
public:
    ESP32Serial(HardwareSerial& ser = Serial, int rxPin = -1, int txPin = -1);
    void begin(uint32_t baudrate = 9600);
    void sendByte(uint8_t data);
    void sendArray(const uint8_t* array, size_t length);
    void sendString(const char* str);
    void sendNumber(uint32_t number, uint8_t length);
    void printf(const char* format, ...);
    
    bool available();
    uint8_t readByte();
    
private:
    HardwareSerial& _serial;
    int _rxPin;
    int _txPin;
    volatile bool _rxFlag = false;
    volatile uint8_t _rxData = 0;
};



extern ESP32Serial SerialEx;

#endif