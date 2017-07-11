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
    UAV_COMMAND_IDENTIFY    = 0x00,
    UAV_COMMAND_READ        = 0x40,
    UAV_COMMAND_WRITE       = 0x80,
    UAV_COMMAND_EXTENDED    = 0xC0,
} uavInterconnectCommand_e;

#define UAV_INTERCONNECT_MAX_SLOTS  64
#define UIB_BIT_RATE            128000
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

static uavInterconnectSlot_t    slots[UAV_INTERCONNECT_MAX_SLOTS];
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

static void sendCommand(timeUs_t currentTimeUs, uavInterconnectCommand_e command, uint8_t slot, uint8_t * buf, int size)
{
    slotStartTimeUs = currentTimeUs;
    serialWrite(busPort, command | slot);
    serialWriteBuf(busPort, buf, size);
}

static void sendDiscover(timeUs_t currentTimeUs, uint8_t slot, uint8_t devId)
{
    uint8_t buf[2] = { UAV_COMMAND_IDENTIFY | slot, devId };
    slotStartTimeUs = currentTimeUs;
    serialWriteBuf(busPort, buf, 2);
}

static uint8_t uavCalculateCRC(uint8_t * buf, int size)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < size; i++) {
        crc = crc8_dvb_s2(crc, buf[i]);
    }
    return crc;
}

static int uavInterconnectFindEmptySlot(void)
{
    for (int i = 0; i < UAV_INTERCONNECT_MAX_SLOTS; i++) {
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
        // Identify command (7 bytes)
        //      FC:     IDENTIFY[1] + DevID[1]
        //      DEV:    PollInterval[2] + Flags[2] + CRC[1]
        case UAV_COMMAND_IDENTIFY:
            if (slotDataBufferCount == 7) {
                if (uavCalculateCRC(&slotDataBuffer[0], 6) == slotDataBuffer[6]) {
                    // CRC valid - process valid IDENTIFY slot
                    slots[slot].allocated = true;
                    slots[slot].rxDataReady = false;
                    slots[slot].txDataReady = false;
                    slots[slot].deviceAddress = slotDataBuffer[1];
                    slots[slot].deviceFlags = slotDataBuffer[4] << 8 | slotDataBuffer[5];
                    slots[slot].pollIntervalUs = (slotDataBuffer[2] << 8 | slotDataBuffer[3]) * 1000;
                    slots[slot].lastPollTimeUs = currentTimeUs;

                    // Find next slot for discovery process
                    discoverySlot = uavInterconnectFindEmptySlot();
                }

                // Regardless of CRC validity - discard buffer data
                slotDataBufferCount = 0;
            }
            break;

        // Read command (18 bytes)
        //      FC:     READ[1]
        //      DEV:    Data[16] + CRC[1]
        case UAV_COMMAND_READ:
            if (slotDataBufferCount == (UIB_PACKET_SIZE + 2)) {
                if (uavCalculateCRC(&slotDataBuffer[0], UIB_PACKET_SIZE + 1) == slotDataBuffer[UIB_PACKET_SIZE + 1]) {
                    // CRC valid - process valid READ slot
                    // TODO
                }

                // Regardless of CRC validity - discard buffer data
                slotDataBufferCount = 0;
            }
            break;
            
        default:
            break;
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
                switchState(STATE_IDLE);
            }
            break;

        case STATE_IDLE:
            // Find highest priority device and read it
            break;
    }
}

void uavInterconnectInit(void)
{
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

#endif
