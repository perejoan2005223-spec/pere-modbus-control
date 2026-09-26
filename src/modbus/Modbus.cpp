#include "Modbus.h"
#include "pico/platform/panic.h"

Modbus::Modbus(PicoOsUart* uart)
{
    // keep the UART we will use to talk to the devices
    this->uart = uart;

    // only one task at a time can communicate through this Modbus object
    this->semaphore = xSemaphoreCreateMutex();

    // if we cannot create the mutex, stop here
    if (this->semaphore == nullptr)
        panic("could not create semaphore");
}

bool Modbus::writeSingleRegister(uint8_t device_address, uint16_t register_address,
                                 uint16_t value)
{
    // take the mutex before sending anything
    if (xSemaphoreTake(this->semaphore, portMAX_DELAY) != pdTRUE)
        return false;

    // send the request and wait for the reply while we still have the mutex
    const bool success = writeRegisterTransaction(device_address, register_address, value);

    // we get here even if the helper returned false, so we always release the mutex
    xSemaphoreGive(this->semaphore);
    return success;
}

bool Modbus::readRegisters(uint8_t device_address, uint8_t function_code,
                          uint16_t register_address, uint16_t* values, uint16_t count)
{
    // we need between 1 and 125 registers and an array to save them
    if (count < 1 || count > 125 || values == nullptr)
        return false;

    // 0x03 reads holding registers, 0x04 reads input registers
    if (function_code != 0x03 && function_code != 0x04)
        return false;

    // take the mutex for the whole request and reply
    if (xSemaphoreTake(this->semaphore, portMAX_DELAY) != pdTRUE)
        return false;

    // the helper does the reading and tells us if it worked
    const bool success = readRegistersTransaction(
        device_address, function_code, register_address, values, count);

    // release it even if something failed inside the helper
    xSemaphoreGive(this->semaphore);
    return success;
}

void Modbus::buildRequest(uint8_t* buffer, uint8_t device_address, uint8_t function_code,
                          uint16_t register_address, uint16_t value_or_count)
{
    buffer[0] = device_address; // which device we want to talk to
    buffer[1] = function_code; // what we want the device to do

    // the register address needs 2 bytes, high first and then low
    buffer[2] = register_address >> 8; // high byte
    buffer[3] = register_address & 0xFF; // low byte

    // for writing this is the value, for reading it is the number of registers
    buffer[4] = value_or_count >> 8; // high byte
    buffer[5] = value_or_count & 0xFF; // low byte

    // calculate using the first 6 bytes, the last 2 will hold the CRC itself
    const uint16_t crc = calculateCRC(buffer, 6);
    buffer[6] = crc & 0xFF; // first byte low of crc
    buffer[7] = crc >> 8; // then byte high of crc
}

bool Modbus::writeRegisterTransaction(uint8_t device_address, uint16_t register_address,
                                      uint16_t value)
{
    // 0x06 means write a single register
    uint8_t buffer[8]{};
    buildRequest(buffer, device_address, 0x06, register_address, value);

    // put all 8 bytes in the UART send queue
    if (uart->write(buffer, sizeof(buffer)) != sizeof(buffer))
        return false;

    // wait for the full reply, this UART uses a 100 ms timeout for each byte
    uint8_t response[8]{};
    if (uart->read(response, sizeof(response), 100) != sizeof(response))
        return false;

    // a normal 0x06 reply repeats what we sent, including the CRC
    for (unsigned int i = 0; i < sizeof(response); i++)
    {
        if (response[i] != buffer[i])
            return false;
    }

    return true;
}

bool Modbus::readRegistersTransaction(uint8_t device_address, uint8_t function_code,
                                     uint16_t register_address, uint16_t* values, uint16_t count)
{
    // ask for count registers, starting at register_address
    uint8_t buffer[8]{};
    buildRequest(buffer, device_address, function_code, register_address, count);

    // if we cannot queue the whole request, return to the public method
    if (uart->write(buffer, sizeof(buffer)) != sizeof(buffer))
        return false;

    // maximum reply: 3 header bytes + 125 registers of 2 bytes + 2 CRC bytes
    uint8_t response[255]{};
    if (!receiveRegisterResponse(response, device_address, function_code, count))
        return false;

    // only update values if the whole reply is correct
    decodeRegisters(response, values, count);
    return true;
}

bool Modbus::receiveRegisterResponse(uint8_t* response, uint8_t device_address,
                                    uint8_t function_code, uint16_t count)
{
    // first read the device, function and byte count (or error code)
    // 100 is the timeout in ms for each byte, not for the whole reply
    if (uart->read(response, 3, 100) != 3)
        return false;

    // the device adds 0x80 to the function code if it reports an error
    if (response[1] == (function_code | 0x80))
    {
        // try to read the last 2 CRC bytes before returning false
        uart->read(response + 3, 2, 100);
        return false;
    }

    // check that the reply is from the device and function we asked for
    // the number of data bytes must be count * 2 because each register uses 2 bytes
    if (response[0] != device_address || response[1] != function_code ||
        response[2] != count * 2)
        return false;

    // we already have the header, now read the data and the 2 CRC bytes
    // response + 3 saves them after the 3 bytes we already received
    const int missing_bytes = count * 2 + 2;
    if (uart->read(response + 3, missing_bytes, 100) != missing_bytes)
        return false;

    // the CRC starts just after the header and all the register data
    const unsigned int crc_index = count * 2 + 3;
    return hasValidCRC(response, crc_index);
}

// calculate CRC to know if there's an error when the data arrives
uint16_t Modbus::calculateCRC(const uint8_t* data, unsigned int length)
{
    uint16_t crc = 0xFFFF; // start calculation with all bits at 1

    for (unsigned int i = 0; i < length; i++)
    {
        crc = crc ^ data[i]; // include the next byte using XOR

        // do 8 steps for this byte, one for each bit
        for (int j = 0; j < 8; j++)
        {
            // check the last bit before shifting
            if ((crc & 1) != 0)
            {
                // if it is 1, shift right and then XOR with 0xA001
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                // if it is 0, just shift right
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

bool Modbus::hasValidCRC(const uint8_t* response, unsigned int crc_index)
{
    // the CRC arrives low byte first, move the high byte 8 bits to join them
    const uint16_t received_crc = response[crc_index] | (response[crc_index + 1] << 8);

    // calculate only up to crc_index, without including the received CRC bytes
    return received_crc == calculateCRC(response, crc_index);
}

void Modbus::decodeRegisters(const uint8_t* response, uint16_t* values, uint16_t count)
{
    // skip the 3 header bytes and take one pair of bytes for each register
    for (unsigned int i = 0; i < count; i++)
    {
        // register data comes high byte first, then low
        values[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
    }
}
