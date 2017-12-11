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

#define __BTSTACK_FILE__ "mesh.c"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ble/mesh/adv_bearer.h"
#include "ble/mesh/pb_adv.h"
#include "ble/mesh/beacon.h"
#include "provisioning_device.h"
#include "btstack.h"

const static uint8_t device_uuid[] = { 0x00, 0x1B, 0xDC, 0x08, 0x10, 0x21, 0x0B, 0x0E, 0x0A, 0x0C, 0x00, 0x0B, 0x0E, 0x0A, 0x0C, 0x00 };

static btstack_packet_callback_registration_t hci_event_callback_registration;

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static int counter = 'a';

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    bd_addr_t addr;
    int i;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
                    // dump bd_addr in pts format
                    gap_local_bd_addr(addr);
                    printf("Local addr: %s - ", bd_addr_to_str(addr));
                    for (i=0;i<6;i++) {
                        printf("%02x", addr[i]);
                    }
                    printf("\n");
                    // setup scanning
                    gap_set_scan_parameters(0, 0x300, 0x300);
                    gap_start_scan();
                    break;

                default:
                    break;
            }
            break;
    }
}

static uint8_t message[10];
static void generate_message(void){
    memset(message, counter, sizeof(message));
    counter++;
    if (counter >= 'z') {
        counter = 'a';
    }
}

static void mesh_message_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    if (packet_type != HCI_EVENT_PACKET) return;
    switch(packet[0]){
        case HCI_EVENT_MESH_META:
            switch(packet[2]){
                case MESH_SUBEVENT_CAN_SEND_NOW:
                    generate_message();
                    adv_bearer_send_mesh_message(&message[0], sizeof(message));
                    adv_bearer_request_can_send_now_for_mesh_message();
                    break;
                case MESH_PB_ADV_LINK_OPEN:
                    printf("Provisioner link opened");
                    break;
                default:
                    break;
            }
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            printf("received mesh message\n");
            break;
        default:
            break;
    }
}

static void mesh_unprovisioned_beacon_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    if (packet_type != HCI_EVENT_PACKET) return;
    uint8_t device_uuid[16];
    uint16_t oob;
    const uint8_t * data;
    switch(packet[0]){
        case GAP_EVENT_ADVERTISING_REPORT:
            data = gap_event_advertising_report_get_data(packet);
            memcpy(device_uuid, &packet[15], 16);
            oob = big_endian_read_16(data, 31);
            printf("received unprovisioned device beacon, oob data %x, device uuid: ", oob);
            printf_hexdump(device_uuid, 16);
            pb_adv_create_link(device_uuid);
            break;
        default:
            break;
    }
}

uint8_t      pts_device_uuid[16];
const char * pts_device_uuid_string = "001BDC0810210B0E0A0C000B0E0A0C00";

static int scan_hex_byte(const char * byte_string){
    int upper_nibble = nibble_for_char(*byte_string++);
    if (upper_nibble < 0) return -1;
    int lower_nibble = nibble_for_char(*byte_string);
    if (lower_nibble < 0) return -1;
    return (upper_nibble << 4) | lower_nibble;
}

static int btstack_parse_hex(const char * string, uint16_t len, uint8_t * buffer){
    int i;
    for (i = 0; i < len; i++) {
        int single_byte = scan_hex_byte(string);
        if (single_byte < 0) return 0;
        string += 2;
        buffer[i] = (uint8_t)single_byte;
        // don't check seperator after last byte
        if (i == len - 1) {
            return 1;
        }
        // optional seperator
        char separator = *string;
        if (separator == ':' && separator == '-' && separator == ' ') {
            string++;
        }
    }
    return 1;
}

static void btstack_print_hex(const uint8_t * data, uint16_t len, char separator){
    int i;
    for (i=0;i<len;i++){
        printf("%02x", data[i]);
        if (separator){
            printf("%c", separator);
        }
    }
    printf("\n");
}

static uint8_t adv_prov_invite_pdu[] = { 0x00, 0x00 };
static uint8_t adv_prov_start_pdu[] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00}; 
static uint8_t adv_prov_public_key_pdu[65];

static uint8_t      prov_static_oob_data[16];
static const char * prov_static_oob_string = "00000000000000000102030405060708";

static void stdin_process(char cmd){
    switch (cmd){
        case '1':
            adv_bearer_request_can_send_now_for_mesh_message();
            break;
        case '2':
            printf("Creating link to device uuid: ");
            printf_hexdump(pts_device_uuid, 16);
            pb_adv_create_link(pts_device_uuid);
            break;
        case '3':
            printf("Close link\n");
            pb_adv_close_link(1, 0);
            break;
        case '4':
            printf("Send invite with attention timer = 0\n");
            pb_adv_send_pdu(adv_prov_invite_pdu, sizeof(adv_prov_invite_pdu));
            break;
        case '5':
            printf("Send Start\n");
            pb_adv_send_pdu(adv_prov_start_pdu, sizeof(adv_prov_start_pdu));
            break;
        case '6':
            printf("Send Public key\n");
            adv_prov_public_key_pdu[0] = 0x03;
            memset(&adv_prov_public_key_pdu[1], 0x5a, 64);
            pb_adv_send_pdu(adv_prov_public_key_pdu, sizeof(adv_prov_public_key_pdu));
            break;
        case 'o':
            printf("+ Output OOB Enabled\n");
            provisioning_device_set_output_oob_actions(0x08, 0x08);
            break;
        case 'i':
            printf("+ Input OOB Enabled\n");
            provisioning_device_set_input_oob_actions(0x08, 0x08);
            break;
        case 's':
            printf("+ Static OOB Enabled\n");
            btstack_parse_hex(prov_static_oob_string, 16, prov_static_oob_data);
            provisioning_device_set_static_oob(16, prov_static_oob_data);
            break;
        default:
            printf("Command: '%c'\n", cmd);
            break;
    }
}

int btstack_main(void);
int btstack_main(void)
{
    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // console
    btstack_stdin_setup(stdin_process);

    // crypto
    btstack_crypto_init();

    // 
    sm_init();

    // mesh
    adv_bearer_init();
    adv_bearer_register_for_mesh_message(&mesh_message_handler);

    beacon_init(device_uuid, 0);
    beacon_register_for_unprovisioned_device_beacons(&mesh_unprovisioned_beacon_handler);
    
    // Provisioning in device role
    provisioning_device_init(device_uuid);

    //
    btstack_parse_hex(pts_device_uuid_string, 16, pts_device_uuid);
    btstack_print_hex(pts_device_uuid, 16, 0);

    // turn on!
	hci_power_control(HCI_POWER_ON);
	    
    return 0;
}
/* EXAMPLE_END */
