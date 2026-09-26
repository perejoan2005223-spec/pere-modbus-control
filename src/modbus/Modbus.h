#pragma once
#include <cstdint>
#include "Uart/PicoOsUart.h"
#include "FreeRTOS.h"
#include "semphr.h"

class Modbus
{
public:
    // give Modbus the UART that we want to use
    Modbus(PicoOsUart* uart);

    // write one value to one register and check the reply
    bool writeSingleRegister(uint8_t device_address, uint16_t register_address, uint16_t value);

    // read count registers, starting at register_address
    // values is where we save them, so the array needs space for at least count values
    bool readRegisters(uint8_t device_address, uint8_t function_code,
                       uint16_t register_address, uint16_t* values, uint16_t count);

private:
    PicoOsUart* uart; // UART used to send and receive the bytes
    SemaphoreHandle_t semaphore; // mutex so tasks sharing this object take turns

    // prepare the 8 bytes we send, including the CRC
    void buildRequest(uint8_t* buffer, uint8_t device_address, uint8_t function_code,
                      uint16_t register_address, uint16_t value_or_count);

    // these helpers do the communication, the public methods already took the mutex
    // they return true or false, then the public methods release the mutex
    bool writeRegisterTransaction(uint8_t device_address, uint16_t register_address,
                                  uint16_t value);
    bool readRegistersTransaction(uint8_t device_address, uint8_t function_code,
                                  uint16_t register_address, uint16_t* values, uint16_t count);

    // read the reply and check the device, function, size and CRC
    bool receiveRegisterResponse(uint8_t* response, uint8_t device_address,
                                 uint8_t function_code, uint16_t count);

    // calculate the CRC from length bytes
    uint16_t calculateCRC(const uint8_t* data, unsigned int length);

    // compare our calculated CRC with the one in the reply
    bool hasValidCRC(const uint8_t* response, unsigned int crc_index);

    // join each pair of bytes and save the registers in values
    void decodeRegisters(const uint8_t* response, uint16_t* values, uint16_t count);
};
