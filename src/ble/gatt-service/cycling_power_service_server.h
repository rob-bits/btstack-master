/*
 * Copyright (C) 2018 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */
#ifndef __CYCLING_POWER_SERVICE_SERVER_H
#define __CYCLING_POWER_SERVICE_SERVER_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

/**
 * Implementation of the GATT Cycling Power Service Server 
 */

// *****************************************************************************
/* GATT_SERVICE_SERVER_START(cycling_power_service_server){Cycling PowerService} 
 *
 */
// *****************************************************************************
/* GATT_SERVICE_SERVER_END */

/* API_START */

typedef enum {
	CYCLING_POWER_SENSOR_LOCATION_OTHER,
	CYCLING_POWER_SENSOR_LOCATION_TOP_OF_SHOE,
	CYCLING_POWER_SENSOR_LOCATION_IN_SHOE,
	CYCLING_POWER_SENSOR_LOCATION_HIP,
	CYCLING_POWER_SENSOR_LOCATION_FRONT_WHEEL,
	CYCLING_POWER_SENSOR_LOCATION_LEFT_CRANK,
	CYCLING_POWER_SENSOR_LOCATION_RIGHT_CRANK,
	CYCLING_POWER_SENSOR_LOCATION_LEFT_PEDAL,
	CYCLING_POWER_SENSOR_LOCATION_RIGHT_PEDAL,
	CYCLING_POWER_SENSOR_LOCATION_FRONT_HUB,
	CYCLING_POWER_SENSOR_LOCATION_REAR_DROPOUT,
	CYCLING_POWER_SENSOR_LOCATION_CHAINSTAY,
	CYCLING_POWER_SENSOR_LOCATION_REAR_WHEEL,
	CYCLING_POWER_SENSOR_LOCATION_REAR_HUB,
	CYCLING_POWER_SENSOR_LOCATION_CHEST,
	CYCLING_POWER_SENSOR_LOCATION_SPIDER,
	CYCLING_POWER_SENSOR_LOCATION_CHAIN_RING,
	CYCLING_POWER_SENSOR_LOCATION_RESERVED
} cycling_power_sensor_location_t;

typedef enum {
	CYCLING_POWER_FLAG_PEDAL_POWER_BALANCE_PRESENT = 0,
	CYCLING_POWER_FLAG_PEDAL_POWER_BALANCE_REFERENCE, // 0 - unknown, 1 - left
	CYCLING_POWER_FLAG_ACCUMULATED_TORQUE_PRESENT,
	CYCLING_POWER_FLAG_ACCUMULATED_TORQUE_SOURCE, // 0 - wheel based, 1 - crank based
	CYCLING_POWER_FLAG_WHEEL_REVOLUTION_DATA_PRESENT,
	CYCLING_POWER_FLAG_CRANK_REVOLUTION_DATA_PRESENT,
	CYCLING_POWER_FLAG_EXTREME_FORCE_MAGNITUDES_PRESENT,
	CYCLING_POWER_FLAG_EXTREME_TORQUE_MAGNITUDES_PRESENT,
	CYCLING_POWER_FLAG_EXTREME_ANGLES_PRESENT,
	CYCLING_POWER_FLAG_TOP_DEAD_SPOT_ANGLE_PRESENT,
	CYCLING_POWER_FLAG_BOTTOM_DEAD_SPOT_ANGLE_PRESENT,
	CYCLING_POWER_FLAG_ACCUMULATED_ENERGY_PRESENT,
	CYCLING_POWER_FLAG_OFFSET_COMPENSATION_INDICATOR,
	CYCLING_POWER_FLAG_RESERVED
} cycling_power_flag_bit_t;

/**
 * @brief Init Server with ATT DB
 */
void cycling_power_service_server_init(void);

/**
 * @brief Update heart rate (unit: beats per minute)
 * @note triggers notifications if subscribed
 */
void cycling_power_service_server_update_values(void);

/* API_END */

#if defined __cplusplus
}
#endif

#endif

