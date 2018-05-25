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

#define __BTSTACK_FILE__ "cycling_power_server_test.c"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cycling_power_server_test.h"
#include "btstack.h"

// https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.gap.appearance.xml
// cycling / cycling power sensor
static const uint16_t appearance = (18 << 6) | 4;

const uint8_t adv_data[] = {
    // Flags general discoverable, BR/EDR not supported
    0x02, 0x01, 0x06, 
    // Name
    0x0E, 0x09, 'C', 'y', 'c', 'l', 'i', 'n', 'g', '_', 'P', 'o', 'w', 'e', 'r',
    // 16-bit Service UUIDs
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, ORG_BLUETOOTH_SERVICE_CYCLING_POWER & 0xff, ORG_BLUETOOTH_SERVICE_CYCLING_POWER >> 8,
    // Appearance
    3, BLUETOOTH_DATA_TYPE_APPEARANCE, appearance & 0xff, appearance >> 8
};

const uint8_t adv_data_len = sizeof(adv_data);

#ifdef HAVE_BTSTACK_STDIN

static void show_usage(void){
    bd_addr_t      iut_address;
    gap_local_bd_addr(iut_address);
    printf("\n--- Bluetooth Cycling Power Server Test Console %s ---\n", bd_addr_to_str(iut_address));
    printf("u    - push update\n");
    printf("\n");
    printf("t    - add positive torque\n");
    printf("T    - add negative torque\n");
    printf("w    - add wheel revolution\n");
    printf("c    - add crank revolution\n");
    printf("e    - add energy\n");
    printf("\n");
    printf("p    - set instantaneous power\n");
    printf("b    - set power balance percentage\n");
    printf("m    - set force magnitude\n");
    printf("M    - set torque magnitude\n");
    printf("a    - set angle\n");
    printf("x    - set top dead spot angle\n");
    printf("y    - set bottom dead spot angle\n");
    printf("\n");
    printf("Ctrl-c - exit\n");
    printf("---\n");
}

static uint16_t event_time_s = 0;
static uint16_t force_magnitude_newton = 0;
static uint16_t torque_magnitude_newton_m = 0;
static uint16_t angle_deg = 0;

static void stdin_process(char cmd){

    switch (cmd){
        case 'u':
            printf("push update\n");
            cycling_power_service_server_update_values();
            event_time_s++;
            break;
    
        case 't':
            printf("add positive torque\n");
            cycling_power_service_server_add_torque(100);
            break;
        case 'T':
            printf("add negative torque\n");
            cycling_power_service_server_add_torque(-100);
            break;
        case 'w':
            printf("add wheel revolution\n");
            cycling_power_service_server_add_wheel_revolution(10, event_time_s);
            break;
        case 'W':
            printf("reverse wheel revolution\n");
            cycling_power_service_server_add_wheel_revolution(-10, event_time_s);
            break;

        case 'c':
            printf("add crank revolution\n");
            cycling_power_service_server_add_crank_revolution(10, event_time_s);
            break;
        case 'e':
            printf("add energy\n");
            cycling_power_service_add_energy(100);
            break;

        case 'p':
            printf("set instantaneous power\n");
            cycling_power_service_server_set_instantaneous_power(100);
            break;
        case 'b':
            printf("set power balance percentage\n");
            cycling_power_service_server_set_pedal_power_balance(50);
            break;
        case 'm':
            force_magnitude_newton += 10;
            printf("set force magnitude\n");
            cycling_power_service_server_set_force_magnitude(force_magnitude_newton-5, force_magnitude_newton+5);
            break;
        case 'M':
            torque_magnitude_newton_m += 10;
            printf("set torque magnitude\n");
            cycling_power_service_server_set_torque_magnitude(torque_magnitude_newton_m-5, torque_magnitude_newton_m+5);
            break;
        case 'a':
            angle_deg += 10;
            printf("set angle\n");
            cycling_power_service_server_set_angle(angle_deg);
            break;
        case 'x':
            printf("set top dead spot angle\n");
            cycling_power_service_server_set_top_dead_spot_angle(180); 
            break;
        case 'y':
            printf("set bottom dead spot angle\n");
            cycling_power_service_server_set_bottom_dead_spot_angle(20); 
            break;

        case '\n':
        case '\r':
            break;
        default:
            show_usage();
            break;
    }
}
#endif

int btstack_main(void);
int btstack_main(void){
    l2cap_init();

    // setup le device db
    le_device_db_init();

    // setup SM: Display only
    sm_init();

    // setup ATT server
    att_server_init(profile_data, NULL, NULL);    

    // setup heart rate service
    // cycling_power_service_server_init(0x2FFFFF, 0x1F, CP_PEDAL_POWER_BALANCE_REFERENCE_LEFT);

    uint32_t feature_flags = 0;   
    feature_flags |= (1 << CP_FEATURE_FLAG_PEDAL_POWER_BALANCE_SUPPORTED);
    feature_flags |= (1 << CP_FEATURE_FLAG_ACCUMULATED_TORQUE_SUPPORTED);
    feature_flags |= (1 << CP_FEATURE_FLAG_ACCUMULATED_TORQUE_SUPPORTED);
    feature_flags |= (1 << CP_FEATURE_FLAG_ACCUMULATED_TORQUE_SUPPORTED);
    feature_flags |= (1 << CP_FEATURE_FLAG_WHEEL_REVOLUTION_DATA_SUPPORTED);
    feature_flags |= (1 << CP_FEATURE_FLAG_CRANK_REVOLUTION_DATA_SUPPORTED);
    
    feature_flags |= (1 << CP_FEATURE_FLAG_EXTREME_MAGNITUDES_SUPPORTED);
    feature_flags |= (CP_SENSOR_MEASUREMENT_CONTEXT_FORCE << CP_FEATURE_FLAG_SENSOR_MEASUREMENT_CONTEXT);
    
    //feature_flags |= (1 << CP_FEATURE_FLAG_EXTREME_ANGLES_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_TOP_AND_BOTTOM_DEAD_SPOT_ANGLE_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_ACCUMULATED_ENERGY_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_OFFSET_COMPENSATION_INDICATOR_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_OFFSET_COMPENSATION_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_CYCLING_POWER_MEASUREMENT_CHARACTERISTIC_CONTENT_MASKING_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_MULTIPLE_SENSOR_LOCATIONS_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_CRANK_LENGTH_ADJUSTMENT_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_CHAIN_LENGTH_ADJUSTMENT_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_CHAIN_WEIGHT_ADJUSTMENT_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_SPAN_LENGTH_ADJUSTMENT_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_INSTANTANEOUS_MEASUREMENT_DIRECTION_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_FACTORY_CALIBRATION_DATE_SUPPORTED);
    //feature_flags |= (1 << CP_FEATURE_FLAG_ENHANCED_OFFSET_COMPENSATION_SUPPORTED);
    //feature_flags |= (2 << CP_FEATURE_FLAG_DISTRIBUTE_SYSTEM_SUPPORT;


    cycling_power_service_server_init(feature_flags, 0x1F, CP_PEDAL_POWER_BALANCE_REFERENCE_LEFT, CP_TORQUE_SOURCE_WHEEL);

    cycling_power_service_server_add_torque(100);
    cycling_power_service_server_add_torque(-100);
    cycling_power_service_server_add_wheel_revolution(10, event_time_s);
    cycling_power_service_server_add_crank_revolution(10, event_time_s);
    cycling_power_service_add_energy(100);

    cycling_power_service_server_set_instantaneous_power(100);
    cycling_power_service_server_set_pedal_power_balance(50);
    force_magnitude_newton += 10;
    cycling_power_service_server_set_force_magnitude(force_magnitude_newton-5, force_magnitude_newton+5);
    torque_magnitude_newton_m += 10;
    cycling_power_service_server_set_torque_magnitude(torque_magnitude_newton_m-5, torque_magnitude_newton_m+5);
    angle_deg += 10;
    cycling_power_service_server_set_angle(angle_deg);
    cycling_power_service_server_set_top_dead_spot_angle(180); 
    cycling_power_service_server_set_bottom_dead_spot_angle(20); 

    // setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
    gap_advertisements_enable(1);

#ifdef HAVE_BTSTACK_STDIN
    btstack_stdin_setup(stdin_process);
#endif
    // turn on!
	hci_power_control(HCI_POWER_ON);
	    
    return 0;
}
/* EXAMPLE_END */
