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

#define MESH_PROV_LINK_OPEN              0x00
#define MESH_PROV_LINK_ACK               0x01
#define MESH_PROV_LINK_CLOSE             0x02

typedef enum mesh_gpcf_format {
    MESH_GPCF_TRANSACTION_START = 0,
    MESH_GPCF_TRANSACTION_ACK,
    MESH_GPCF_TRANSACTION_CONT,
    MESH_GPCF_PROV_BEARER_CONTROL,
} mesh_gpcf_format_t;

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
static uint8_t  pbv_adv_send_ack;

static uint8_t  prov_msg_seg_total; // total segments
static uint8_t  prov_msg_seg_next;  // next expected segment 
static uint16_t prov_msg_len;   // 
static uint16_t prov_msg_pos;
static uint8_t  prov_msg_fcs;
static uint8_t  prov_msg_buffer[100];   // TODO: how large are prov messages?

static void provisioning_handle_bearer_control(uint32_t link_id, uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
    uint8_t bearer_opcode = pdu[0] >> 2;
    uint8_t reason;
    switch (bearer_opcode){
        case MESH_PROV_LINK_OPEN: // Open a session on a bearer with a device
            // does it match our device_uuid?
            if (memcmp(&pdu[1], device_uuid, 16) != 0) break;
            switch(link_state){
                case LINK_STATE_W4_OPEN:
                    pbv_adv_link_id = link_id;
                    pbv_adv_transaction_nr = 0xff;  // first transaction nr will be 0x00 
                    log_info("link open %08x", pbv_adv_link_id);
                    printf("Link Open - id %08x. we're the unprovisioned device\n", pbv_adv_link_id);
                    link_state = LINK_STATE_W2_SEND_ACK;
                    adv_bearer_request_can_send_now_for_pb_adv();
                    break;
                case LINK_STATE_W2_SEND_ACK:
                    break;
                case LINK_STATE_OPEN:
                    if (pbv_adv_link_id != link_id) break;
                    printf("Link Open - resend ACK\n");
                    link_state = LINK_STATE_W2_SEND_ACK;
                    adv_bearer_request_can_send_now_for_pb_adv();
                    break;
            }
            break;
        case MESH_PROV_LINK_ACK:   // Acknowledge a session on a bearer
            break;
        case MESH_PROV_LINK_CLOSE: // Close a session on a bearer
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

static void provisioning_send_ack(void){
    pbv_adv_send_ack = 1;
    adv_bearer_request_can_send_now_for_pb_adv();
}

static void provisioning_handle_pdu(void){

    // TODO: check FCS

    // TODO: check message
    uint8_t type = prov_msg_buffer[0];
    printf("Prov MSG Type %x\n", type);
    printf_hexdump(prov_msg_buffer, prov_msg_len);

    // TODO: check msg len

    // ACK message
    provisioning_send_ack();

    // TODO: dispatch

    // TODO: reset state
    prov_msg_len = 0;

}

static void provisioning_transaction_start(uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){

    // check if previous transation incomplete
    if (prov_msg_len){
        printf("previous transaction not completed\n");
        return;
    }
    if (transaction_nr == pbv_adv_transaction_nr){
        printf("packet from previous transaction\n");
        return;
    }
    pbv_adv_transaction_nr = transaction_nr;

    // setup buffer
    prov_msg_len       = big_endian_read_16(pdu, 1);
    prov_msg_seg_total = pdu[0] >> 2;
    prov_msg_fcs       = pdu[3];
    prov_msg_seg_next  = 1;

    uint16_t payload_len = size - 4;
    memcpy(prov_msg_buffer, &pdu[4], payload_len);
    prov_msg_pos = payload_len;

    // complete?
    if (prov_msg_seg_total == 0){
        provisioning_handle_pdu();
    }
}

static void provisioning_transaction_cont(uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
     // check segment index
     uint8_t seg = pdu[0] >> 2;
     if (seg != prov_msg_seg_next) return;
     // check transaction nr
    if (transaction_nr == pbv_adv_transaction_nr){
        printf("wrong transaction nr\n");
        return;
    }
     uint16_t remaining_space = sizeof(prov_msg_buffer) - prov_msg_pos;
     uint16_t fragment_size   = size - 1;
     if (fragment_size > remaining_space) return;
     memcpy(&prov_msg_buffer[prov_msg_pos], &pdu[1], fragment_size);

     // complete
     if (prov_msg_pos == prov_msg_len && prov_msg_seg_total == seg){
        provisioning_handle_pdu();
     }
}

static void provisioning_transaction_ack(uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
    printf("provisioning_transaction_ack. trans %u: ", transaction_nr);
    printf_hexdump(pdu, size);
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
            mesh_gpcf_format_t generic_provisioning_control_format = (mesh_gpcf_format_t) generic_provisioning_control & 3;
            if (generic_provisioning_control_format != MESH_GPCF_PROV_BEARER_CONTROL){
                // verify link id and link state
                if (link_id    != pbv_adv_link_id) break;
                if (link_state != LINK_STATE_OPEN) break;
            }
            switch (generic_provisioning_control_format){
                case MESH_GPCF_TRANSACTION_START:
                    provisioning_transaction_start(transaction_nr, &data[7], size-7);
                    break;
                case MESH_GPCF_TRANSACTION_CONT:
                    provisioning_transaction_cont(transaction_nr, &data[7], size-7);
                    break;
                case MESH_GPCF_TRANSACTION_ACK:
                    provisioning_transaction_ack(transaction_nr, &data[7], size-7);
                    break;
                case MESH_GPCF_PROV_BEARER_CONTROL:
                    provisioning_handle_bearer_control(link_id, transaction_nr, &data[7], size-7);
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
                        buffer[4] = 0;
                        buffer[5] = (1 << 2) | 3; // Link Ack | Provisioning Bearer Control
                        adv_bearer_send_pb_adv(buffer, sizeof(buffer));
                        log_info("link ack %08x", pbv_adv_link_id);
                        printf("Sending Link Ack\n");
                    }
                    if (pbv_adv_send_ack){
                        pbv_adv_send_ack = 0;
                        uint8_t buffer[6];
                        big_endian_store_32(buffer, 0, pbv_adv_link_id);
                        buffer[4] = pbv_adv_transaction_nr;
                        buffer[5] = 1; // Transaction Ack ;
                        adv_bearer_send_pb_adv(buffer, sizeof(buffer));
                        log_info("transaction ack %08x", pbv_adv_link_id);
                        printf("Sending Transaction Ack\n");
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
