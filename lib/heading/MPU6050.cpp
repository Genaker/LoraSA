/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "MPU6050.h"
#include "../comms/bus.h"

#include <pgmspace.h>

/* ================================================================ *
 | Default MotionApps v6.12 28-byte FIFO packet structure:          |
 |                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  |
 |                                                                  |
 | [ACC  X][ACC  Y][ACC  Z][GYRO X][GYRO Y][GYRO Z]					|
 |  16  17  18  19  20  21  22  23  24  25  26  27					|
 * ================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)

// this divisor is pre configured into the above image and can't be modified at this time.
#ifndef MPU6050_DMP_FIFO_RATE_DIVISOR
#define MPU6050_DMP_FIFO_RATE_DIVISOR                                                    \
    0x01 // The New instance of the Firmware has this as the default
#endif

// this is the most basic initialization I can create. with the intent that we access the
// register bytes as few times as needed to get the job done. for detailed descriptins of
// all registers and there purpose google "MPU-6000/MPU-6050 Register Map and
// Descriptions"
uint8_t MPU6050::dmpInitialize()
{ // Lets get it over with fast Write everything once and set it up nicely
    uint8_t val;
    uint16_t ival;
    // Reset procedure per instructions in the "MPU-6000/MPU-6050 Register Map and
    // Descriptions" page 41
    I2Cdev::writeBit(wireObj, devAddr, 0x6B, 7,
                     (val = 1)); // PWR_MGMT_1: reset with 100ms delay
    delay(100);
    I2Cdev::writeBits(wireObj, devAddr, 0x6A, 2, 3,
                      (val = 0b111)); // full SIGNAL_PATH_RESET: with another 100ms delay
    delay(100);
    I2Cdev::writeBytes(
        wireObj, devAddr, 0x6B, 1,
        &(val = 0x01)); // 1000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
    I2Cdev::writeBytes(wireObj, devAddr, 0x38, 1,
                       &(val = 0x00)); // 0000 0000 INT_ENABLE: no Interrupt
    I2Cdev::writeBytes(
        wireObj, devAddr, 0x23, 1,
        &(val = 0x00)); // 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
    I2Cdev::writeBytes(
        wireObj, devAddr, 0x1C, 1,
        &(val = 0x00)); // 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
    I2Cdev::writeBytes(
        wireObj, devAddr, 0x37, 1,
        &(val = 0x80)); // 1001 0000 INT_PIN_CFG: ACTL The logic level for int pin is
                        // active low. and interrupt status bits are cleared on any read
    I2Cdev::writeBytes(
        wireObj, devAddr, 0x6B, 1,
        &(val = 0x01)); // 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
    I2Cdev::writeBytes(
        wireObj, devAddr, 0x19, 1,
        &(val = 0x04)); // 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz (
                        // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
    I2Cdev::writeBytes(wireObj, devAddr, 0x1A, 1,
                       &(val = 0x01)); // 0000 0001 CONFIG: Digital Low Pass Filter (DLPF)
                                       // Configuration 188HZ
                                       // //Im betting this will be the beat
    if (!writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE))
        return 1; // Loads the DMP image into the MPU6050 Memory // Should Never Fail
    I2Cdev::writeWords(wireObj, devAddr, 0x70, 1,
                       &(ival = 0x0400)); // DMP Program Start Address
    I2Cdev::writeBytes(wireObj, devAddr, 0x1B, 1,
                       &(val = 0x18)); // 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
    I2Cdev::writeBytes(wireObj, devAddr, 0x6A, 1,
                       &(val = 0xC0)); // 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
    I2Cdev::writeBytes(wireObj, devAddr, 0x38, 1,
                       &(val = 0x02)); // 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
    I2Cdev::writeBit(wireObj, devAddr, 0x6A, 2,
                     1); // Reset FIFO one last time just for kicks. (MPUi2cWrite reads
                         // 0x6A first and only alters 1 bit and then saves the byte)

    setDMPEnabled(false); // disable DMP for compatibility with the MPU6050 library
                          /*
                              dmpPacketSize += 16;//DMP_FEATURE_6X_LP_QUAT
                              dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_ACCEL
                              dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_GYRO
                          */
    dmpPacketSize = 28;
    return 0;
}

void MPU6050::setDMPEnabled(bool e)
{
    dmpEnabled = e;
    I2Cdev::writeBit(wireObj, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT,
                     e);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy,
                         int16_t *gz)
{
    uint8_t buffer[14];

    I2Cdev::readBytes(wireObj, devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer,
                      I2Cdev::readTimeout);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

void MPU6050::setMemoryBank(uint8_t bank)
{
    bank &= 0x1F;
    I2Cdev::writeBytes(wireObj, devAddr, MPU6050_RA_BANK_SEL, 1, &bank);
}

void MPU6050::setMemoryStartAddress(uint8_t address)
{
    I2Cdev::writeBytes(wireObj, devAddr, MPU6050_RA_MEM_START_ADDR, 1, &address);
}

bool MPU6050::writeProgMemoryBlock(const unsigned char *dump, size_t sz, uint8_t bank,
                                   uint8_t address, bool verify)
{
    setMemoryBank(bank);
    setMemoryStartAddress(address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer = 0;
    uint8_t *progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    uint16_t i;
    uint8_t j;
    if (verify)
        verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);

    for (i = 0; i < sz;)
    {
        // determine correct chunk size according to bank position and data size
        chunkSize = min(MPU6050_DMP_MEMORY_CHUNK_SIZE, min((int)sz - i, 256 - address));

        // write the chunk of data as specified
        for (j = 0; j < chunkSize; j++)
            progBuffer[j] = pgm_read_byte(dump + i + j);
        I2Cdev::writeBytes(wireObj, devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

        // verify data if needed
        if (verify && verifyBuffer)
        {
            setMemoryBank(bank);
            setMemoryStartAddress(address);
            I2Cdev::readBytes(wireObj, devAddr, MPU6050_RA_MEM_R_W, chunkSize,
                              verifyBuffer, I2Cdev::readTimeout);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0)
            {
                free(verifyBuffer);
                free(progBuffer);
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < sz)
        {
            if (address == 0)
                bank++;
            setMemoryBank(bank);
            setMemoryStartAddress(address);
        }
    }
    if (verify)
        free(verifyBuffer);
    free(progBuffer);
    return true;
}

/** Get timeout to get a packet from FIFO buffer.
 * @return Current timeout to get a packet from FIFO buffer
 * @see MPU6050_FIFO_DEFAULT_TIMEOUT
 */
uint32_t MPU6050::getFIFOTimeout() { return MPU6050_FIFO_DEFAULT_TIMEOUT; }

int8_t MPU6050::getFIFOBytes(uint8_t *data, uint8_t length)
{
    if (length > 0)
    {
        return I2Cdev::readBytes(wireObj, devAddr, MPU6050_RA_FIFO_R_W, length, data,
                                 I2Cdev::readTimeout);
    }

    *data = 0;
    return 0;
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPU6050::getFIFOCount()
{
    uint16_t buffer;
    I2Cdev::readBytes(wireObj, devAddr, MPU6050_RA_FIFO_COUNTH, 2, (uint8_t *)&buffer,
                      I2Cdev::readTimeout);
    return (buffer << 8) | ((buffer >> 8) & 0xff);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPU6050::resetFIFO()
{
    I2Cdev::writeBit(wireObj, devAddr, MPU6050_RA_USER_CTRL,
                     MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}

/** Get latest byte from FIFO buffer no matter how much time has passed.
 * ===                  GetCurrentFIFOPacket                    ===
 * ================================================================
 * Returns 1) when nothing special was done
 *         2) when recovering from overflow
 *         0) when no valid data is available
 * ================================================================ */
uint8_t MPU6050::getCurrentFIFOPacket(MPU6050Reading &v)
{
    uint8_t length = dmpPacketSize;
    int16_t fifoC;
    // This section of code is for when we allowed more than 1 packet to be acquired
    uint32_t BreakTimer = micros();
    bool packetReceived = false;
    do
    {
        if ((fifoC = getFIFOCount()) > length)
        {

            if (fifoC > 200)
            { // if you waited to get the FIFO buffer to > 200 bytes it will take longer
              // to get the last packet in the FIFO Buffer than it will take to  reset the
              // buffer and wait for the next to arrive
                resetFIFO(); // Fixes any overflow corruption
                fifoC = 0;
                while (!(fifoC = getFIFOCount()) &&
                       ((micros() - BreakTimer) <= (getFIFOTimeout())))
                    ; // Get Next New Packet
            }
            else
            { // We have more than 1 packet but less than 200 bytes of data in the FIFO
              // Buffer
                uint8_t Trash[I2CDEVLIB_WIRE_BUFFER_LENGTH];
                while ((fifoC = getFIFOCount()) > length)
                { // Test each time just in case the MPU is writing to the FIFO Buffer
                    fifoC = fifoC - length; // Save the last packet
                    uint16_t RemoveBytes;
                    while (fifoC)
                    { // fifo count will reach zero so this is safe
                        RemoveBytes =
                            (fifoC < I2CDEVLIB_WIRE_BUFFER_LENGTH)
                                ? fifoC
                                : I2CDEVLIB_WIRE_BUFFER_LENGTH; // Buffer Length is
                                                                // different than the
                                                                // packet length this will
                                                                // efficiently clear the
                                                                // buffer
                        getFIFOBytes(Trash, (uint8_t)RemoveBytes);
                        fifoC -= RemoveBytes;
                    }
                }
            }
        }
        if (!fifoC)
            return 0; // Called too early no data or we timed out after FIFO Reset
        // We have 1 packet
        packetReceived = fifoC == length;
        if (!packetReceived && (micros() - BreakTimer) > (getFIFOTimeout()))
            return 0;
    } while (!packetReceived);
    getFIFOBytes(fifoPacket, length); // Get 1 packet

    v.q.w = (((uint32_t)fifoPacket[0] << 24) | ((uint32_t)fifoPacket[1] << 16) |
             ((uint32_t)fifoPacket[2] << 8) | fifoPacket[3]);
    v.q.x = (((uint32_t)fifoPacket[4] << 24) | ((uint32_t)fifoPacket[5] << 16) |
             ((uint32_t)fifoPacket[6] << 8) | fifoPacket[7]);
    v.q.y = (((uint32_t)fifoPacket[8] << 24) | ((uint32_t)fifoPacket[9] << 16) |
             ((uint32_t)fifoPacket[10] << 8) | fifoPacket[11]);
    v.q.z = (((uint32_t)fifoPacket[12] << 24) | ((uint32_t)fifoPacket[13] << 16) |
             ((uint32_t)fifoPacket[14] << 8) | fifoPacket[15]);

    v.a.x = (fifoPacket[16] << 8) | fifoPacket[17];
    v.a.y = (fifoPacket[18] << 8) | fifoPacket[19];
    v.a.z = (fifoPacket[20] << 8) | fifoPacket[21];

    v.g.x = (fifoPacket[22] << 8) | fifoPacket[23];
    v.g.y = (fifoPacket[24] << 8) | fifoPacket[25];
    v.g.z = (fifoPacket[26] << 8) | fifoPacket[27];
    return 1;
}
