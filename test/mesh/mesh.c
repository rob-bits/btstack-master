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
#include "ble/mesh/beacon.h"

#include "btstack.h"

static btstack_packet_callback_registration_t hci_event_callback_registration;

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static int counter = 'a';

const uint8_t adv_data[] = {
    // Flags general discoverable, BR/EDR not supported
    0x02, 0x01, 0x06, 
    // Name
    0x0b, 0x09, 'L', 'E', ' ', 'C', 'o', 'u', 'n', 't', 'e', 'r', 
};
const uint8_t adv_data_len = sizeof(adv_data);

const static uint8_t device_uuid[] = { 0x00, 0x1B, 0xDC, 0x08, 0x10, 0x21, 0x0B, 0x0E, 0x0A, 0x0C, 0x00, 0x0B, 0x0E, 0x0A, 0x0C, 0x00 };

/* LISTING_START(packetHandler): Packet Handler */
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
                    // request to send
                    // adv_bearer_request_can_send_now_for_mesh_message();
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
            memcpy(device_uuid, &packet[3], 16);
            oob = big_endian_read_16(data, 19);
            printf("received unprovisioned device beacon, oob data %x, device uuid: ", oob);
            printf_hexdump(device_uuid, 16);
            break;
        default:
            break;
    }
}

// Provisioning Bearer Conrol

typedef enum {
    LINK_STATE_W4_OPEN,
    LINK_STATE_W2_SEND_ACK,
    LINK_STATE_OPEN,
} link_state_t;
static link_state_t link_state;
static uint32_t pbv_adv_link_id;
static uint8_t  pbv_adv_transaction_nr;

static void provisioning_handle_bearer_control(uint32_t link_id, uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
    uint8_t bearer_opcode = pdu[0] >> 2;
    uint8_t reason;
    switch (bearer_opcode){
        case 0: // Link Open - Open a session on a bearer with a device
            // link active?
            if (link_state != LINK_STATE_W4_OPEN) break;
            // does it match our device_uuid?
            if (memcmp(&pdu[1], device_uuid, 16) != 0) break;
            printf("Link Open\n");
            link_state = LINK_STATE_W2_SEND_ACK;
            pbv_adv_link_id = link_id;
            pbv_adv_transaction_nr = transaction_nr; 
            adv_bearer_request_can_send_now_for_pb_adv();
            break;
        case 1: // Link Ack  - Acknowledge a session on a bearer
            break;
        case 2: // Link Close - Close a session on a bearer
            // does it match link id
            if (link_id != pbv_adv_link_id) break;
            reason = pdu[1];
            link_state = LINK_STATE_W4_OPEN;
            if (reason > 0x02){
                printf("Link Close with reason %x\n", reason);
            } else {
                printf("Link Close with unrecognized reason %x\n", reason);
            }
            break;
        default:
            printf("BearerOpcode %x reserved for future use\n", bearer_opcode);
            break;
    }

}

// static void provisioning_run(void){
// }

static void pb_adv_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    if (packet_type != HCI_EVENT_PACKET) return;
    const uint8_t * data;
    uint32_t link_id;
    uint8_t  transaction_nr;
    uint8_t  generic_provisioning_control;
    switch(packet[0]){
        case GAP_EVENT_ADVERTISING_REPORT:
            data = gap_event_advertising_report_get_data(packet);
            // PDB ADV PDU
            link_id = big_endian_read_32(data, 2);
            transaction_nr = data[6];
            // generic provision PDU
            generic_provisioning_control = data[7];
            uint8_t generic_provisioning_control_format = generic_provisioning_control & 3;
            switch (generic_provisioning_control_format){
                case 3: // Provisioning Bearer Control
                    provisioning_handle_bearer_control(link_id, transaction_nr, &data[7], size-7);
                    break;
                default: 
                    printf("Received pb_adv pdu: link id %08x, transaction %u: control %x \n", link_id, transaction_nr, generic_provisioning_control);
                    break;
            }
            break;
        case HCI_EVENT_MESH_META:
            switch(packet[2]){
                case MESH_SUBEVENT_CAN_SEND_NOW:
                    if (link_state == LINK_STATE_W2_SEND_ACK){
                        link_state = LINK_STATE_OPEN;
                        // build packet
                        uint8_t buffer[6];
                        big_endian_store_32(buffer, 0, pbv_adv_link_id);
                        buffer[4] = pbv_adv_transaction_nr;
                        buffer[5] = (1 << 2) | 3; // Link Ack | Provisioning Bearer Control
                        adv_bearer_send_pb_adv(buffer, sizeof(buffer));
                        printf("Sending Link Ack\n");
                    }
                    break;
                default:
                    break;
            }
        default:
            break;
    }
}

static void stdin_process(char cmd){
    switch (cmd){
        case '1':
            adv_bearer_request_can_send_now_for_mesh_message();
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

    // setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
    gap_advertisements_enable(1);

    // console
    btstack_stdin_setup(stdin_process);

    // mesh
    adv_bearer_init();
    adv_bearer_register_for_mesh_message(&mesh_message_handler);

    beacon_init(device_uuid, 0);
    beacon_register_for_unprovisioned_device_beacons(&mesh_unprovisioned_beacon_handler);
    
    adv_bearer_register_for_pb_adv(&pb_adv_handler);

    // turn on!
	hci_power_control(HCI_POWER_ON);
	    
    return 0;
}
/* EXAMPLE_END */
