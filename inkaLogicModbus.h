#ifndef INKA_LOGIC_MODBUS_H
#define INKA_LOGIC_MODBUS_H
#include <cstdint>
#include "driver/uart.h"
#include "esp_modbus_common.h"
#include "mbcontroller.h"

class InkaLogicModbus
{
public:
    InkaLogicModbus();
    void config(uart_port_t port, int tx_pin, int rx_pin, int rts_pin);
    esp_err_t begin(uint8_t slave_id, int baud_rate);
    void changeSlave(uint8_t slave_id);
    void idle(void (*callback)());
    void preTransmission(void (*callback)());
    void postTransmission(void (*callback)());
    uint16_t getResponseBuffer(uint8_t index);
    void     clearResponseBuffer();
    esp_err_t setTransmitBuffer(uint8_t index, uint16_t value);
    void     clearTransmitBuffer();
    void beginTransmission(uint16_t address);
    void send(uint8_t data);
    void send(uint16_t data);
    void send(uint32_t data);
    uint8_t available();
    uint16_t receive();
    esp_err_t readCoils(uint16_t address, uint16_t qty);
    esp_err_t readDiscreteInputs(uint16_t address, uint16_t qty);
    esp_err_t readHoldingRegisters(uint16_t address, uint16_t qty);
    esp_err_t readInputRegisters(uint16_t address, uint16_t qty);
    esp_err_t writeSingleCoil(uint16_t address, uint8_t state);
    esp_err_t writeSingleRegister(uint16_t address, uint16_t value);
    esp_err_t writeMultipleCoils(uint16_t address, uint16_t qty);
    esp_err_t writeMultipleCoils();
    esp_err_t writeMultipleRegisters(uint16_t address, uint16_t qty);
    esp_err_t writeMultipleRegisters();
    esp_err_t maskWriteRegister(uint16_t address, uint16_t andMask, uint16_t orMask);
    esp_err_t readWriteMultipleRegisters(uint16_t readAddr, uint16_t readQty, uint16_t writeAddr, uint16_t writeQty);
    esp_err_t readWriteMultipleRegisters(uint16_t readAddr, uint16_t readQty);
private:
    uart_port_t _port;
    int         _tx_pin;
    int         _rx_pin;
    int         _rts_pin;
    void*       _master_handler;
    bool        _initialized;
    uint8_t _u8MBSlave;
    static const uint8_t ku8MaxBufferSize = 64;
    uint16_t _u16ResponseBuffer[ku8MaxBufferSize];
    uint8_t  _u8ResponseBufferIndex;
    uint8_t  _u8ResponseBufferLength;
    uint16_t _u16TransmitBuffer[ku8MaxBufferSize];
    uint8_t  _u8TransmitBufferIndex;
    uint16_t _u16TransmitBufferLength;
    uint16_t _u16ReadAddress;
    uint16_t _u16ReadQty;
    uint16_t _u16WriteAddress;
    uint16_t _u16WriteQty;
    static const uint8_t ku8MBReadCoils                  = 0x01;
    static const uint8_t ku8MBReadDiscreteInputs         = 0x02;
    static const uint8_t ku8MBReadHoldingRegisters       = 0x03;
    static const uint8_t ku8MBReadInputRegisters         = 0x04;
    static const uint8_t ku8MBWriteSingleCoil            = 0x05;
    static const uint8_t ku8MBWriteSingleRegister        = 0x06;
    static const uint8_t ku8MBWriteMultipleCoils         = 0x0F;
    static const uint8_t ku8MBWriteMultipleRegisters     = 0x10;
    static const uint8_t ku8MBMaskWriteRegister          = 0x16;
    static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17;
    static const uint16_t ku16MBResponseTimeout = 1000;
    esp_err_t executeTransaction(uint8_t u8MBFunction);
    esp_err_t mapEspError(esp_err_t err);
    void (*_idle)();
    void (*_preTransmission)();
    void (*_postTransmission)();
};
#endif