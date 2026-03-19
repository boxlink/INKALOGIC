#include "inkaLogicModbus.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "InkaLogicModbus";

InkaLogicModbus::InkaLogicModbus()
{
    _port           = UART_NUM_2;
    _tx_pin         = UART_PIN_NO_CHANGE;
    _rx_pin         = UART_PIN_NO_CHANGE;
    _rts_pin        = UART_PIN_NO_CHANGE;
    _master_handler = NULL;
    _initialized    = false;
    _u8MBSlave      = 1;
    _u8TransmitBufferIndex = 0;
    _u16TransmitBufferLength = 0;
    _u8ResponseBufferIndex = 0;
    _u8ResponseBufferLength = 0;
    _u16ReadAddress  = 0;
    _u16ReadQty      = 0;
    _u16WriteAddress = 0;
    _u16WriteQty     = 0;
    _idle             = 0;
    _preTransmission  = 0;
    _postTransmission = 0;
    memset(_u16ResponseBuffer, 0, sizeof(_u16ResponseBuffer));
    memset(_u16TransmitBuffer, 0, sizeof(_u16TransmitBuffer));
}

void InkaLogicModbus::config(uart_port_t port, int tx_pin, int rx_pin, int rts_pin)
{
    _port    = port;
    _tx_pin  = tx_pin;
    _rx_pin  = rx_pin;
    _rts_pin = rts_pin;
}

esp_err_t InkaLogicModbus::begin(uint8_t slave_id, int baud_rate)
{
    _u8MBSlave = slave_id;
    _u8TransmitBufferIndex = 0;
    _u16TransmitBufferLength = 0;
    _u8ResponseBufferIndex = 0;
    _u8ResponseBufferLength = 0;

    mb_communication_info_t comm = {
        .mode       = MB_MODE_RTU,
        .slave_addr = 1,
        .port       = _port,
        .baudrate   = (uint32_t)baud_rate,
        .parity     = UART_PARITY_DISABLE,
    };

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &_master_handler);
    if (err != ESP_OK) return ESP_FAIL;

    err = mbc_master_setup((void*)&comm);
    if (err != ESP_OK) return ESP_FAIL;

    err = uart_set_pin(_port, _tx_pin, _rx_pin, _rts_pin, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return ESP_FAIL;

    err = mbc_master_start();
    if (err != ESP_OK) return ESP_FAIL;

    err = uart_set_mode(_port, UART_MODE_RS485_HALF_DUPLEX);
    if (err != ESP_OK) return ESP_FAIL;

    _initialized = true;
    return ESP_OK;
}

void InkaLogicModbus::changeSlave(uint8_t slave_id) { _u8MBSlave = slave_id; }
void InkaLogicModbus::idle(void (*callback)())             { _idle = callback; }
void InkaLogicModbus::preTransmission(void (*callback)())  { _preTransmission = callback; }
void InkaLogicModbus::postTransmission(void (*callback)()) { _postTransmission = callback; }

uint16_t InkaLogicModbus::getResponseBuffer(uint8_t index)
{
    if (index < ku8MaxBufferSize) return _u16ResponseBuffer[index];
    return 0xFFFF;
}

void InkaLogicModbus::clearResponseBuffer()
{
    for (uint8_t i = 0; i < ku8MaxBufferSize; i++) _u16ResponseBuffer[i] = 0;
    _u8ResponseBufferIndex = 0;
    _u8ResponseBufferLength = 0;
}

esp_err_t InkaLogicModbus::setTransmitBuffer(uint8_t index, uint16_t value)
{
    if (index < ku8MaxBufferSize)
    {
        _u16TransmitBuffer[index] = value;
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

void InkaLogicModbus::clearTransmitBuffer()
{
    for (uint8_t i = 0; i < ku8MaxBufferSize; i++) _u16TransmitBuffer[i] = 0;
    _u8TransmitBufferIndex = 0;
    _u16TransmitBufferLength = 0;
}

void InkaLogicModbus::beginTransmission(uint16_t address)
{
    _u16WriteAddress = address;
    _u8TransmitBufferIndex = 0;
    _u16TransmitBufferLength = 0;
}

void InkaLogicModbus::send(uint16_t data)
{
    if (_u8TransmitBufferIndex < ku8MaxBufferSize)
    {
        _u16TransmitBuffer[_u8TransmitBufferIndex++] = data;
        _u16TransmitBufferLength = _u8TransmitBufferIndex;
    }
}

void InkaLogicModbus::send(uint32_t data)
{
    send((uint16_t)(data & 0xFFFF));
    send((uint16_t)((data >> 16) & 0xFFFF));
}

void InkaLogicModbus::send(uint8_t data) { send((uint16_t)data); }

uint8_t InkaLogicModbus::available()
{
    return _u8ResponseBufferLength - _u8ResponseBufferIndex;
}

uint16_t InkaLogicModbus::receive()
{
    if (_u8ResponseBufferIndex < _u8ResponseBufferLength)
        return _u16ResponseBuffer[_u8ResponseBufferIndex++];
    return 0xFFFF;
}

esp_err_t InkaLogicModbus::readCoils(uint16_t address, uint16_t qty)
{
    _u16ReadAddress = address;
    _u16ReadQty = qty;
    return executeTransaction(ku8MBReadCoils);
}

esp_err_t InkaLogicModbus::readDiscreteInputs(uint16_t address, uint16_t qty)
{
    _u16ReadAddress = address;
    _u16ReadQty = qty;
    return executeTransaction(ku8MBReadDiscreteInputs);
}

esp_err_t InkaLogicModbus::readHoldingRegisters(uint16_t address, uint16_t qty)
{
    _u16ReadAddress = address;
    _u16ReadQty = qty;
    return executeTransaction(ku8MBReadHoldingRegisters);
}

esp_err_t InkaLogicModbus::readInputRegisters(uint16_t address, uint16_t qty)
{
    _u16ReadAddress = address;
    _u16ReadQty = qty;
    return executeTransaction(ku8MBReadInputRegisters);
}

esp_err_t InkaLogicModbus::writeSingleCoil(uint16_t address, uint8_t state)
{
    _u16WriteAddress = address;
    _u16WriteQty = (state ? 0xFF00 : 0x0000);
    return executeTransaction(ku8MBWriteSingleCoil);
}

esp_err_t InkaLogicModbus::writeSingleRegister(uint16_t address, uint16_t value)
{
    _u16WriteAddress = address;
    _u16WriteQty = 0;
    _u16TransmitBuffer[0] = value;
    return executeTransaction(ku8MBWriteSingleRegister);
}

esp_err_t InkaLogicModbus::writeMultipleCoils(uint16_t address, uint16_t qty)
{
    _u16WriteAddress = address;
    _u16WriteQty = qty;
    return executeTransaction(ku8MBWriteMultipleCoils);
}

esp_err_t InkaLogicModbus::writeMultipleCoils()
{
    _u16WriteQty = _u16TransmitBufferLength;
    return executeTransaction(ku8MBWriteMultipleCoils);
}

esp_err_t InkaLogicModbus::writeMultipleRegisters(uint16_t address, uint16_t qty)
{
    _u16WriteAddress = address;
    _u16WriteQty = qty;
    return executeTransaction(ku8MBWriteMultipleRegisters);
}

esp_err_t InkaLogicModbus::writeMultipleRegisters()
{
    _u16WriteQty = _u8TransmitBufferIndex;
    return executeTransaction(ku8MBWriteMultipleRegisters);
}

esp_err_t InkaLogicModbus::maskWriteRegister(uint16_t address, uint16_t andMask, uint16_t orMask)
{
    _u16WriteAddress = address;
    _u16TransmitBuffer[0] = andMask;
    _u16TransmitBuffer[1] = orMask;
    return executeTransaction(ku8MBMaskWriteRegister);
}

esp_err_t InkaLogicModbus::readWriteMultipleRegisters(uint16_t readAddr, uint16_t readQty,
                                                     uint16_t writeAddr, uint16_t writeQty)
{
    _u16ReadAddress  = readAddr;
    _u16ReadQty      = readQty;
    _u16WriteAddress = writeAddr;
    _u16WriteQty     = writeQty;
    return executeTransaction(ku8MBReadWriteMultipleRegisters);
}

esp_err_t InkaLogicModbus::readWriteMultipleRegisters(uint16_t readAddr, uint16_t readQty)
{
    _u16ReadAddress = readAddr;
    _u16ReadQty     = readQty;
    _u16WriteQty    = _u8TransmitBufferIndex;
    return executeTransaction(ku8MBReadWriteMultipleRegisters);
}

esp_err_t InkaLogicModbus::mapEspError(esp_err_t err)
{
    return err;
}

esp_err_t InkaLogicModbus::executeTransaction(uint8_t u8MBFunction)
{
    esp_err_t u8MBStatus = ESP_OK;

    if (!_initialized) return ESP_FAIL;
    if (_u8MBSlave < 1 || _u8MBSlave > 247) return ESP_ERR_INVALID_ARG;

    if (_preTransmission) _preTransmission();

    esp_err_t err = ESP_FAIL;

    switch (u8MBFunction)
    {
        case ku8MBReadCoils:
        case ku8MBReadDiscreteInputs:
        {
            uint8_t rawBuffer[ku8MaxBufferSize * 2] = {0};
            uint16_t byteCount = (_u16ReadQty + 7) / 8;

            mb_param_request_t request = {
                .slave_addr = _u8MBSlave,
                .command    = u8MBFunction,
                .reg_start  = _u16ReadAddress,
                .reg_size   = _u16ReadQty
            };

            err = mbc_master_send_request(&request, (void*)rawBuffer);
            u8MBStatus = mapEspError(err);

            if (u8MBStatus == ESP_OK)
            {
                _u8ResponseBufferLength = 0;
                for (uint16_t i = 0; i < byteCount && i < ku8MaxBufferSize; i++)
                {
                    _u16ResponseBuffer[i] = (uint16_t)rawBuffer[i];
                    _u8ResponseBufferLength++;
                }
            }
            break;
        }

        case ku8MBReadHoldingRegisters:
        case ku8MBReadInputRegisters:
        {
            mb_param_request_t request = {
                .slave_addr = _u8MBSlave,
                .command    = u8MBFunction,
                .reg_start  = _u16ReadAddress,
                .reg_size   = _u16ReadQty
            };

            err = mbc_master_send_request(&request, (void*)_u16ResponseBuffer);
            u8MBStatus = mapEspError(err);

            if (u8MBStatus == ESP_OK)
                _u8ResponseBufferLength = (_u16ReadQty < ku8MaxBufferSize) ? _u16ReadQty : ku8MaxBufferSize;
            break;
        }

        case ku8MBReadWriteMultipleRegisters:
        {
            mb_param_request_t writeReq = {
                .slave_addr = _u8MBSlave,
                .command    = 0x10,
                .reg_start  = _u16WriteAddress,
                .reg_size   = _u16WriteQty
            };

            err = mbc_master_send_request(&writeReq, (void*)_u16TransmitBuffer);
            u8MBStatus = mapEspError(err);
            if (u8MBStatus != ESP_OK) break;

            mb_param_request_t readReq = {
                .slave_addr = _u8MBSlave,
                .command    = 0x03,
                .reg_start  = _u16ReadAddress,
                .reg_size   = _u16ReadQty
            };

            err = mbc_master_send_request(&readReq, (void*)_u16ResponseBuffer);
            u8MBStatus = mapEspError(err);

            if (u8MBStatus == ESP_OK)
                _u8ResponseBufferLength = (_u16ReadQty < ku8MaxBufferSize) ? _u16ReadQty : ku8MaxBufferSize;
            break;
        }

        case ku8MBWriteSingleCoil:
        {
            uint16_t coilValue = _u16WriteQty;

            mb_param_request_t request = {
                .slave_addr = _u8MBSlave,
                .command    = 0x05,
                .reg_start  = _u16WriteAddress,
                .reg_size   = 1
            };

            err = mbc_master_send_request(&request, (void*)&coilValue);
            u8MBStatus = mapEspError(err);
            break;
        }

        case ku8MBWriteSingleRegister:
        {
            mb_param_request_t request = {
                .slave_addr = _u8MBSlave,
                .command    = 0x06,
                .reg_start  = _u16WriteAddress,
                .reg_size   = 1
            };

            err = mbc_master_send_request(&request, (void*)&_u16TransmitBuffer[0]);
            u8MBStatus = mapEspError(err);
            break;
        }

        case ku8MBWriteMultipleCoils:
        {
            uint8_t rawCoils[ku8MaxBufferSize] = {0};
            uint16_t byteCount = (_u16WriteQty + 7) / 8;

            for (uint16_t b = 0; b < byteCount; b++)
            {
                uint8_t coilByte = 0;
                for (uint8_t bit = 0; bit < 8; bit++)
                {
                    uint16_t idx = b * 8 + bit;
                    if (idx < _u16WriteQty && (_u16TransmitBuffer[idx] & 0x0001))
                        coilByte |= (1 << bit);
                }
                rawCoils[b] = coilByte;
            }

            mb_param_request_t request = {
                .slave_addr = _u8MBSlave,
                .command    = 0x0F,
                .reg_start  = _u16WriteAddress,
                .reg_size   = _u16WriteQty
            };

            err = mbc_master_send_request(&request, (void*)rawCoils);
            u8MBStatus = mapEspError(err);
            break;
        }

        case ku8MBWriteMultipleRegisters:
        {
            mb_param_request_t request = {
                .slave_addr = _u8MBSlave,
                .command    = 0x10,
                .reg_start  = _u16WriteAddress,
                .reg_size   = _u16WriteQty
            };

            err = mbc_master_send_request(&request, (void*)_u16TransmitBuffer);
            u8MBStatus = mapEspError(err);
            break;
        }

        case ku8MBMaskWriteRegister:
        {
            uint16_t regValue = 0;

            mb_param_request_t readReq = {
                .slave_addr = _u8MBSlave,
                .command    = 0x03,
                .reg_start  = _u16WriteAddress,
                .reg_size   = 1
            };

            err = mbc_master_send_request(&readReq, (void*)&regValue);
            u8MBStatus = mapEspError(err);
            if (u8MBStatus != ESP_OK) break;

            regValue = (regValue & _u16TransmitBuffer[0]) | (_u16TransmitBuffer[1] & ~_u16TransmitBuffer[0]);

            mb_param_request_t writeReq = {
                .slave_addr = _u8MBSlave,
                .command    = 0x06,
                .reg_start  = _u16WriteAddress,
                .reg_size   = 1
            };

            err = mbc_master_send_request(&writeReq, (void*)&regValue);
            u8MBStatus = mapEspError(err);
            break;
        }

        default:
            u8MBStatus = ESP_ERR_NOT_SUPPORTED;
            break;
    }

    if (_postTransmission) _postTransmission();

    _u8TransmitBufferIndex = 0;
    _u16TransmitBufferLength = 0;
    _u8ResponseBufferIndex = 0;

    return u8MBStatus;
}