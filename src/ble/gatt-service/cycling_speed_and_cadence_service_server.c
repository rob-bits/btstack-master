/*
 * Copyright (C) 2014 BlueKitchen GmbH
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

#define __BTSTACK_FILE__ "cycling_speed_and_cadence_service_server.c"


#include "bluetooth.h"
#include "btstack_defines.h"
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "btstack_util.h"
#include "bluetooth_gatt.h"
#include "btstack_debug.h"
#include "l2cap.h"

#include "ble/gatt-service/cycling_speed_and_cadence_service_server.h"

typedef enum {
	 CSC_FLAG_WHEEL_REVOLUTION_DATA_SUPPORTED = 0,
	 CSC_FLAG_CRANK_REVOLUTION_DATA_SUPPORTED,
	 CSC_FLAG_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED
} cycling_speed_and_cadence_service_flag_bit_t;

typedef struct {
	hci_con_handle_t con_handle;

	// characteristic: CSC Mesurement 
	uint16_t measurement_value_handle;
	uint8_t  wheel_revolution_data_supported;
	uint8_t  crank_revolution_data_supported;
	uint32_t cumulative_wheel_revolutions;
	uint16_t last_wheel_event_time; // Unit has a resolution of 1/1024s
	uint32_t cumulative_crank_revolutions;
	uint16_t last_crank_event_time; // Unit has a resolution of 1/1024s
	
	// characteristic descriptor: Client Characteristic Configuration
	uint16_t measurement_client_configuration_descriptor_handle;
	uint16_t measurement_client_configuration_descriptor_notify;
	btstack_context_callback_registration_t measurement_callback;

	// sensor locations bitmap
	uint16_t sensor_location_value_handle;
	cycling_speed_and_cadence_body_sensor_location_t sensor_location;
	
	// characteristic: Heart Rate Control Point
	uint16_t control_point_value_handle;
	uint16_t control_point_client_configuration_descriptor_handle;
	uint16_t control_point_client_configuration_descriptor_indicate;
	btstack_context_callback_registration_t control_point_callback;

} cycling_speed_and_cadence_t;

static att_service_handler_t cycling_speed_and_cadence_service;
static cycling_speed_and_cadence_t cycling_speed_and_cadence;

static uint16_t cycling_speed_and_cadence_service_read_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
	UNUSED(con_handle);
	UNUSED(attribute_handle);
	UNUSED(offset);
	cycling_speed_and_cadence_t * instance = &cycling_speed_and_cadence;

	if (attribute_handle == instance->measurement_client_configuration_descriptor_handle){
		if (buffer && buffer_size >= 2){
			little_endian_store_16(buffer, 0, instance->measurement_client_configuration_descriptor_notify);
		} 
		return 2;
	}

	if (attribute_handle == instance->control_point_client_configuration_descriptor_handle){
		printf("control_point_client_configuration_descriptor_handle \n");
		if (buffer && buffer_size >= 2){
			little_endian_store_16(buffer, 0, instance->control_point_client_configuration_descriptor_indicate);
		} 
		return 2;
	}

	return 0;
}

static int cycling_speed_and_cadence_service_write_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
	UNUSED(con_handle);
	UNUSED(transaction_mode);
	UNUSED(offset);
	UNUSED(buffer_size);
	cycling_speed_and_cadence_t * instance = &cycling_speed_and_cadence;

	if (attribute_handle == instance->measurement_client_configuration_descriptor_handle){
		if (buffer_size < 2){
			return ATT_ERROR_INVALID_OFFSET;
		}
		instance->measurement_client_configuration_descriptor_notify = little_endian_read_16(buffer, 0);
		instance->con_handle = con_handle;
		if (instance->measurement_client_configuration_descriptor_notify == 0x01){
			printf("notification enabled\n");
		} else {
			printf("notification disabled\n");
		}
		return 0;
	}

	if (attribute_handle == instance->control_point_client_configuration_descriptor_handle){
		if (buffer_size < 2){
			return ATT_ERROR_INVALID_OFFSET;
		}
		instance->control_point_client_configuration_descriptor_indicate = little_endian_read_16(buffer, 0);
		instance->con_handle = con_handle;
		if (instance->control_point_client_configuration_descriptor_indicate == 0x01){
			printf("indication enabled\n");
		} else {
			printf("indication disabled\n");
		}
		return 0;
	}
	printf("heart_rate_service_read_callback, not handeled read on handle 0x%02x\n", attribute_handle);
	return 0;
}

void cycling_speed_and_cadence_service_server_init(cycling_speed_and_cadence_body_sensor_location_t sensor_location, uint8_t wheel_revolution_data_supported, uint8_t crank_revolution_data_supported){
	cycling_speed_and_cadence_t * instance = &cycling_speed_and_cadence;
	
	instance->wheel_revolution_data_supported = wheel_revolution_data_supported;
	instance->crank_revolution_data_supported = crank_revolution_data_supported;
	instance->sensor_location = sensor_location;

	// get service handle range
	uint16_t start_handle = 0;
	uint16_t end_handle   = 0xffff;
	int service_found = gatt_server_get_get_handle_range_for_service_with_uuid16(ORG_BLUETOOTH_SERVICE_CYCLING_SPEED_AND_CADENCE, &start_handle, &end_handle);
	if (!service_found){
		printf("no service found\n");
		return;
	}
	// // get CSC Mesurement characteristic value handle and client configuration handle
	instance->measurement_value_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_CSC_MEASUREMENT);
	instance->measurement_client_configuration_descriptor_handle = gatt_server_get_client_configuration_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_CSC_MEASUREMENT);
	
	// get Body Sensor Location characteristic value handle and client configuration handle
	instance->sensor_location_value_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_SENSOR_LOCATION);
	
	// get SC Control Point characteristic value handle and client configuration handle
	instance->control_point_value_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_SC_CONTROL_POINT);
	instance->control_point_client_configuration_descriptor_handle = gatt_server_get_client_configuration_handle_for_characteristic_with_uuid16(start_handle, end_handle, ORG_BLUETOOTH_CHARACTERISTIC_SC_CONTROL_POINT);
	printf("Measurement     value handle 0x%02x\n", instance->measurement_value_handle);
	printf("Measurement Cfg value handle 0x%02x\n", instance->measurement_client_configuration_descriptor_handle);
	printf("Sensor location value handle 0x%02x\n", instance->sensor_location_value_handle);
	printf("Control Point   value handle 0x%02x\n", instance->control_point_value_handle);
	printf("Control P. Cfg. value handle 0x%02x\n", instance->control_point_client_configuration_descriptor_handle);
	
	cycling_speed_and_cadence_service.start_handle   = start_handle;
	cycling_speed_and_cadence_service.end_handle     = end_handle;
	cycling_speed_and_cadence_service.read_callback  = &cycling_speed_and_cadence_service_read_callback;
	cycling_speed_and_cadence_service.write_callback = &cycling_speed_and_cadence_service_write_callback;
	
	att_server_register_service_handler(&cycling_speed_and_cadence_service);
}

void cycling_speed_and_cadence_service_server_update_values(void){}