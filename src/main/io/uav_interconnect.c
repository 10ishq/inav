/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

#include "platform.h"
#include "build/build_config.h"

#ifdef USE_UAV_INTERCONNECT

#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"
#include "io/uav_interconnect.h"


typedef enum {
    UIB_COMMAND_IDENTIFY    = 0x00,
    UIB_COMMAND_READ        = 0x40,
    UIB_COMMAND_WRITE       = 0x80,
    UIB_COMMAND_EXTENDED    = 0xC0,
} uavInterconnectCommand_e;

typedef enum {
    UIB_FLAG_HAS_READ       = (1 << 0),     // Device supports READ command (sensor)
    UIB_FLAG_HAS_WRITE      = (1 << 1),     // Device supports WRITE command (sensor configuration or executive device)
} uibDeviceFlags_e;

#define UIB_MAX_DEVICES         256
#define UIB_MAX_SLOTS           16
#define UIB_BIT_RATE            115200
#define UIB_PORT_OPTIONS        (SERIAL_NOT_INVERTED | SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_UNIDIR)
#define UIB_PACKET_SIZE         16

#define UIB_DISCOVERY_DELAY_US  2000000 // 2 seconds from power-up to allow all devices to boot
#define UIB_SLOT_INTERVAL_US    4900    // 4.9ms
#define UIB_GUARD_INTERVAL_US   2000

typedef struct {
    bool        allocated;
    uint8_t     deviceAddress;
    uint16_t    deviceFlags;
    timeUs_t    pollIntervalUs;
    timeUs_t    lastPollTimeUs;

    bool        rxDataReady;
    bool        txDataReady;    
    uint8_t     rxPacket[UIB_PACKET_SIZE];
    uint8_t     txPacket[UIB_PACKET_SIZE];
} uavInterconnectSlot_t;

typedef enum {
    STATE_INITIALIZE,
    STATE_DISCOVER,
    STATE_IDLE,
} uavInterconnectState_t;

static serialPort_t *           busPort;
static bool                     uibInitialized = false;

static uavInterconnectSlot_t *  devices[UIB_MAX_DEVICES];
static uavInterconnectSlot_t    slots[UIB_MAX_SLOTS];
static timeUs_t                 slotStartTimeUs;
static uavInterconnectState_t   busState = STATE_INITIALIZE;

static int                      discoverySlot;
static int                      discoveryAddress;

static uint8_t                  slotDataBuffer[20]; // Max transaction length is 20 bytes
static int                      slotDataBufferCount;
static timeUs_t                 responseLastByteTimeUs;

static void switchState(uavInterconnectState_t newState)
{
    if (busState == newState)
        return;

    busState = newState;
}

static uint8_t uavCalculateCRC(uint8_t * buf, int size)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < size; i++) {
        crc = crc8_dvb_s2(crc, buf[i]);
    }
    return crc;
}

static void sendDiscover(timeUs_t currentTimeUs, uint8_t slot, uint8_t devId)
{
    uint8_t buf[3];
    buf[0] = UIB_COMMAND_IDENTIFY | slot;
    buf[1] = devId;
    buf[2] = uavCalculateCRC(&buf[0], 2);
    slotStartTimeUs = currentTimeUs;
    serialWriteBuf(busPort, buf, 3);
/*
    uint8_t buf[8];
    buf[0] = UIB_COMMAND_IDENTIFY | slot;
    buf[1] = devId;
    buf[2] = uavCalculateCRC(&buf[0], 2);
    buf[3] = 0x00;
    buf[4] = 0x64;
    buf[5] = 0x00;
    buf[6] = UIB_FLAG_HAS_READ | UIB_FLAG_HAS_WRITE;
    buf[7] = uavCalculateCRC(&buf[0], 7);
    slotStartTimeUs = currentTimeUs;
    serialWriteBuf(busPort, buf, 8);
*/
}

static void sendRead(timeUs_t currentTimeUs, uint8_t slot)
{
    uint8_t buf[2];
    buf[0] = UIB_COMMAND_READ | slot;
    buf[1] = uavCalculateCRC(&buf[0], 1);
    slotStartTimeUs = currentTimeUs;
    serialWriteBuf(busPort, buf, 2);
}

static void sendWrite(timeUs_t currentTimeUs, uint8_t slot, uint8_t * data)
{
    uint8_t buf[UIB_PACKET_SIZE + 2];
    buf[0] = UIB_COMMAND_WRITE | slot;
    memcpy(&buf[1], data, UIB_PACKET_SIZE);
    buf[UIB_PACKET_SIZE + 1] = uavCalculateCRC(&buf[0], UIB_PACKET_SIZE + 1);
    slotStartTimeUs = currentTimeUs;
    serialWriteBuf(busPort, buf, UIB_PACKET_SIZE + 2);
}

static int uavInterconnectFindEmptySlot(void)
{
    for (int i = 0; i < UIB_MAX_SLOTS; i++) {
        if (!slots[i].allocated)
            return i;
    }

    return -1;
}

static void uavInterconnectProcessSlot(timeUs_t currentTimeUs)
{
    // First byte is command / slot
    uint8_t cmd = slotDataBuffer[0] & 0xC0;
    uint8_t slot = slotDataBuffer[0] & 0x3F;

    // CRC is calculated over the whole slot, including command byte(s) sent by FC. This ensures integrity of the transaction as a whole
    switch (cmd) {
        // Identify command (8 bytes)
        //      FC:     IDENTIFY[1] + DevID[1] + CRC1[1]
        //      DEV:    PollInterval[2] + Flags[2] + CRC2[1]
        case UIB_COMMAND_IDENTIFY:
            if (slotDataBufferCount == 8) {
                if (uavCalculateCRC(&slotDataBuffer[0], 7) == slotDataBuffer[7]) {
                    // CRC valid - process valid IDENTIFY slot
                    slots[slot].allocated = true;
                    slots[slot].rxDataReady = false;
                    slots[slot].txDataReady = false;
                    slots[slot].deviceAddress = slotDataBuffer[1];
                    slots[slot].deviceFlags = slotDataBuffer[5] << 8 | slotDataBuffer[6];
                    slots[slot].pollIntervalUs = (slotDataBuffer[3] << 8 | slotDataBuffer[4]) * 1000;
                    slots[slot].lastPollTimeUs = currentTimeUs;

                    // Save pointer to slot to be able to address device by DevID internally
                    devices[slots[slot].deviceAddress] = &slots[slot];
                    
                    debug[1]++;
                    debug[2] = slots[slot].deviceAddress;

                    // Find next slot for discovery process
                    discoverySlot = uavInterconnectFindEmptySlot();
                }
                else {
                    debug[3]++;
                }

                // Regardless of CRC validity - discard buffer data
                slotDataBufferCount = 0;
            }
            break;

        // Read command (18 bytes)
        //      FC:     READ[1] + CRC1[1]
        //      DEV:    Data[16] + CRC2[1]
        case UIB_COMMAND_READ:
            if (slotDataBufferCount == (UIB_PACKET_SIZE + 3)) {
                if (uavCalculateCRC(&slotDataBuffer[0], UIB_PACKET_SIZE + 2) == slotDataBuffer[UIB_PACKET_SIZE + 2]) {
                    // CRC valid - process valid READ slot
                    // Check if this slot has read capability and is allocated
                    if (slots[slot].allocated && (slots[slot].deviceFlags & UIB_FLAG_HAS_READ) && !slots[slot].rxDataReady) {
                        memcpy(slots[slot].rxPacket, &slotDataBuffer[2], UIB_PACKET_SIZE);
                        slots[slot].rxDataReady = true;
                    }
                }

                // Regardless of CRC validity - discard buffer data
                slotDataBufferCount = 0;
            }
            break;

        // Write command (18 bytes)
        //      FC:     WRITE[1] + Data[16] + CRC1[1]
        //      DEV:    ACK[1] + CRC2[1]
        case UIB_COMMAND_WRITE:
            if (slotDataBufferCount == (UIB_PACKET_SIZE + 4)) {
                // Don't check for CRC at the moment - just receive the packet
                // Regardless of CRC validity - discard buffer data
                slotDataBufferCount = 0;
            }
            break;

        default:
            break;
    }
}

static void uibProcessScheduledTransactions(timeUs_t currentTimeUs)
{
    int slotPrio = 0x100;
    int slotId = -1;

    // First - find device with highest priority that has the READ capability and is scheduled for READ
    for (int i = 0; i < UIB_MAX_SLOTS; i++) {
        if (!slots[i].allocated) 
            continue;

        if ((slots[i].deviceFlags & UIB_FLAG_HAS_READ) && ((currentTimeUs - slots[i].lastPollTimeUs) >= slots[i].pollIntervalUs) && (slotPrio > slots[i].deviceAddress)) {
            slotId = i;
            slotPrio = slots[i].deviceAddress;
        }
    }

    // READ command
    if (slotId >= 0) {
        sendRead(currentTimeUs, slotId);
        slots[slotId].lastPollTimeUs = currentTimeUs;
        return;
    }

    // No READ command executed - check if we have data to WRITE
    slotPrio = 0x100;
    slotId = -1;

    for (int i = 0; i < UIB_MAX_SLOTS; i++) {
        if (!slots[i].allocated) 
            continue;

        if (slots[i].txDataReady && (slots[i].deviceFlags & UIB_FLAG_HAS_WRITE) && (slotPrio > slots[i].deviceAddress)) {
            slotId = i;
            slotPrio = slots[i].deviceAddress;
        }
    }

    // WRITE command
    if (slotId >= 0) {
        sendWrite(currentTimeUs, slotId, slots[slotId].txPacket);
        slots[slotId].txDataReady = false;
        return;
    }
}

void uavInterconnectTask(timeUs_t currentTimeUs)
{
    if (!uibInitialized)
        return;

    // Flush receive buffer if guard interval elapsed
    if ((currentTimeUs - responseLastByteTimeUs) >= UIB_GUARD_INTERVAL_US) {
        slotDataBufferCount = 0;
    }

    // Receive bytes to the buffer
    bool hasNewBytes = false;
    while (serialRxBytesWaiting(busPort) > 0) {
        uint8_t c = serialRead(busPort);
        if (slotDataBufferCount < (int)(sizeof(slotDataBuffer) / sizeof(slotDataBuffer[0]))) {
            slotDataBuffer[slotDataBufferCount++] = c;
            hasNewBytes = true;
        }
    }

    // If we have new bytes - process packet
    if (hasNewBytes && slotDataBufferCount >= 7) {  // minimum transaction length is 7 bytes - no point in processing something smaller
        uavInterconnectProcessSlot(currentTimeUs);
    }

    // Process request scheduling - we can initiate another slot if guard interval has elapsed and slot interval has elapsed as well
    bool canSendNewRequest = ((currentTimeUs - responseLastByteTimeUs) >= UIB_GUARD_INTERVAL_US) &&
                             ((currentTimeUs - slotStartTimeUs) >= UIB_SLOT_INTERVAL_US);

    if (!canSendNewRequest)
        return;

    // We get here only if we can send requests - no timeout checking should be done beyond this point
    switch (busState) {
        case STATE_INITIALIZE:
            if ((currentTimeUs - slotStartTimeUs) > UIB_DISCOVERY_DELAY_US) {
                discoverySlot = 0;
                discoveryAddress = 0;
                switchState(STATE_DISCOVER);
            }
            break;

        case STATE_DISCOVER:
            if (discoverySlot >= 0) {
                sendDiscover(currentTimeUs, discoverySlot, discoveryAddress);
                debug[0]++;

                if (discoveryAddress == 0xFF) {
                    // All addresses have been polled
                    switchState(STATE_IDLE);
                }
                else {
                    // Query next address and stick here
                    discoveryAddress++;
                }
            }
            else {
                // All slots are allocated - can't discover more devices
                discoverySlot = 0;
                discoveryAddress = 0;
                switchState(STATE_IDLE);
            }
            break;

        case STATE_IDLE:
            // Find highest priority device and read/write it
            // If no device is ready to be polled at the moment:
            // issue IDENTIFY command for one of the existing devices. This will allow re-discovery of 
            // temporary disconnected devices (due to device intermittent failure)
            uibProcessScheduledTransactions(currentTimeUs);
            break;
    }
}

void uavInterconnectInit(void)
{
    for (int i = 0; i < UIB_MAX_DEVICES; i++) {
        devices[i] = NULL;
    }

    serialPortConfig_t * portConfig = findSerialPortConfig(FUNCTION_UAV_INTERCONNECT);
    if (!portConfig)
        return;

    busPort = openSerialPort(portConfig->identifier, FUNCTION_UAV_INTERCONNECT, NULL, UIB_BIT_RATE, MODE_RXTX, UIB_PORT_OPTIONS);
    if (!busPort)
        return;

    slotStartTimeUs = micros();
    uibInitialized = true;
}

bool uavInterconnectIsInitialized(void)
{
    return uibInitialized;
}

bool uavInterconnect_DeviceDetected(uint8_t devId)
{
    if (!devices[devId])
        return false;

    return devices[devId]->allocated;
}

timeUs_t uavInterconnect_GetPollRateUs(uint8_t devId)
{
    if (!devices[devId])
        return 0;

    return devices[devId]->pollIntervalUs;
}

bool uavInterconnect_DataAvailable(uint8_t devId)
{
    if (!devices[devId])
        return false;

    return devices[devId]->rxDataReady;
}

bool uavInterconnect_Read(uint8_t devId, uint8_t * buffer)
{
    if (!devices[devId])
        return false;

    if (!devices[devId]->rxDataReady)
        return false;

    memcpy(buffer, devices[devId]->rxPacket, UIB_PACKET_SIZE);
    return true;
}

#endif
