#include "Modbus.h"
#include "pico/platform/panic.h"
#include "task.h"

// invented timeouts
namespace
{
    constexpr TickType_t response_timeout = pdMS_TO_TICKS(500);
    constexpr TickType_t drain_timeout = pdMS_TO_TICKS(500);
}

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
    // wait 500ms for another task to release the mutex
    if (xSemaphoreTake(this->semaphore, pdMS_TO_TICKS(500)) != pdTRUE)
        return false;

    // do not send anything until the previous reply has been cleared
    bool success = false;
    if (prepareTransaction())
    {
        success = writeRegisterTransaction(device_address, register_address, value);
        recovery_pending = !success;
    }

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

    // wait 500ms for another task to release the mutex
    if (xSemaphoreTake(this->semaphore, pdMS_TO_TICKS(500)) != pdTRUE)
        return false;

    // keep the mutex during recovery too, so another task cannot send in the middle
    bool success = false;
    if (prepareTransaction())
    {
        success = readRegistersTransaction(device_address, function_code, register_address, values, count);
        recovery_pending = !success;
    }

    // release it even if something failed inside the helper
    xSemaphoreGive(this->semaphore);
    return success;
}

TickType_t Modbus::frameGap() const
{
    // RTU needs at least 3.5 quiet characters between frames
    // use 11 bits per character, which covers our 8N2 setup
    const int baud = uart->get_baud();
    const unsigned int gap_ms = baud > 19200 ? 2u :
        (38500u + static_cast<unsigned int>(baud) - 1u) / static_cast<unsigned int>(baud);

    // round up to ticks and add one, as we might be near the next tick already
    return static_cast<TickType_t>(
        (static_cast<uint64_t>(gap_ms) * configTICK_RATE_HZ + 999u) / 1000u + 1u);
}

bool Modbus::prepareTransaction()
{
    if (uart->get_baud() <= 0)
    {
        recovery_pending = true;
        return false;
    }

    const TickType_t quiet_time = frameGap();

    // after an error, discard for another reply timeout before trying again
    // a single flush would miss bytes that arrive a bit later
    // RTU has no request ID: devices replying even later need a longer wait here
    const TickType_t minimum_wait = recovery_pending ? response_timeout : 0;
    const TickType_t limit = minimum_wait + drain_timeout;
    const TickType_t started_at = xTaskGetTickCount();
    TickType_t last_byte_at = started_at;

    // leave this set if noise prevents us from finishing the recovery
    recovery_pending = true;

    while (true)
    {
        TickType_t now = xTaskGetTickCount();
        const TickType_t elapsed = now - started_at;

        // do not stay here forever if the line keeps receiving garbage
        if (elapsed >= limit)
            return false;

        const TickType_t remaining = limit - elapsed;
        const TickType_t wait_ticks = quiet_time < remaining ? quiet_time : remaining;
        uint8_t discarded = 0;

        // read one byte at a time, so even continuous noise has a time limit
        if (uart->read(&discarded, 1, wait_ticks * portTICK_PERIOD_MS) == 1)
        {
            last_byte_at = xTaskGetTickCount();
            continue;
        }

        now = xTaskGetTickCount();
        if ((TickType_t)(now - started_at) >= minimum_wait && (TickType_t)(now - last_byte_at) >= quiet_time)
        {
            recovery_pending = false;
            return true;
        }
    }
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

    // use one timeout for the whole reply
    const TickType_t start_time = xTaskGetTickCount();

    uint8_t response[8]{};
    if (!readBytes(response, sizeof(response), start_time, response_timeout))
    {
        return false;
    }

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
    // use the same timer for the header, data and CRC
    const TickType_t start_time = xTaskGetTickCount();

    // first read the device, function and byte count (or error code)
    if (!readBytes(response, 3, start_time, response_timeout))
    {
        return false;
    }

    // the device reports an error
    if (response[1] == (function_code | 0x80))
    {
        // try to read the remaining CRC bytes using the time left
        readBytes(response + 3, 2, start_time, response_timeout);
        return false;
    }

    // check the device, function and expected number of data bytes
    if (response[0] != device_address || response[1] != function_code || response[2] != count * 2)
    {
        return false;
    }

    // read the register data and CRC after the header
    const int missing_bytes = count * 2 + 2;
    if (!readBytes(response + 3, missing_bytes, start_time, response_timeout))
    {
        return false;
    }

    // check the CRC before accepting the reply
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

bool Modbus::readBytes(uint8_t* buffer, unsigned int size, TickType_t start_time, TickType_t timeout)
{
    for (unsigned int i = 0; i < size; i++)
    {
        // check how much time we already used
        TickType_t elapsed = xTaskGetTickCount() - start_time;

        if (elapsed >= timeout)
        {
            return false;
        }

        // the UART expects milliseconds, so convert the remaining ticks
        TickType_t remaining_ticks = timeout - elapsed;
        int remaining_ms = remaining_ticks * portTICK_PERIOD_MS;

        // read one byte using only the time we have left
        if (uart->read(buffer + i, 1, remaining_ms) != 1)
        {
            return false;
        }
    }

    // also check the time after receiving the last byte
    TickType_t elapsed = xTaskGetTickCount() - start_time;
    return elapsed <= timeout;
}
