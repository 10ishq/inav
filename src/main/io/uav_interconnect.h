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

#define UIB_DEV_COMPASS         0x08

#define UIB_DEV_BAROMETER       0x11
#define UIB_DEV_RANGEFINDER     0x12

#define UIB_DEV_GPS             0x20
#define UIB_DEV_OPTICAL_FLOW    0x22

#define UIB_DEV_AIRSPEED        0x40

#define UIB_DEV_RC_CONTROL      0x80
#define UIB_DEV_REMOTE_STEERING 0x81    // Processed in special "ROBOT" flight mode

void uavInterconnectTask(timeUs_t currentTimeUs);
void uavInterconnectInit(void);
bool uavInterconnectIsInitialized(void);

bool uavInterconnect_DeviceDetected(uint8_t devId);
timeUs_t uavInterconnect_GetPollRateUs(uint8_t devId);
bool uavInterconnect_DataAvailable(uint8_t devId);
bool uavInterconnect_Read(uint8_t devId, uint8_t * buffer);

#endif
