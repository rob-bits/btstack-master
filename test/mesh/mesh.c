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
#include "classic/rfcomm.h" // for crc8
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

// PB-ADV - Provisioning Bearer using Advertisement Bearer

typedef enum {
    LINK_STATE_W4_OPEN,
    LINK_STATE_W2_SEND_ACK,
    LINK_STATE_OPEN,
} link_state_t;
static link_state_t link_state;

static uint32_t pbv_adv_link_id;
static uint8_t  pbv_adv_transaction_nr_incoming;
static uint8_t  pbv_adv_transaction_nr_outgoing;
static uint8_t  pbv_adv_send_ack;
static uint8_t  pbv_adv_send_msg;

static uint8_t  prov_msg_seg_total; // total segments
static uint8_t  prov_msg_seg_next;  // next expected segment 
static uint16_t prov_msg_len;   // 
static uint16_t prov_msg_pos;
static uint8_t  prov_msg_fcs;
static uint8_t  prov_msg_buffer[100];   // TODO: how large are prov messages?
static uint16_t prov_msg_out_len;
static uint16_t prov_msg_out_pos;
static uint8_t  prov_msg_out_seg;

static void (*pbv_adv_packet_handler)(void);

static void pbv_adv_register_handler(void (*packet_handler)(void)){
    pbv_adv_packet_handler = packet_handler;
}

static void pbv_adv_handle_bearer_control(uint32_t link_id, uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
    uint8_t bearer_opcode = pdu[0] >> 2;
    uint8_t reason;
    switch (bearer_opcode){
        case MESH_PROV_LINK_OPEN: // Open a session on a bearer with a device
            // does it match our device_uuid?
            if (memcmp(&pdu[1], device_uuid, 16) != 0) break;
            switch(link_state){
                case LINK_STATE_W4_OPEN:
                    pbv_adv_link_id = link_id;
                    pbv_adv_transaction_nr_incoming = 0xff;  // first transaction nr will be 0x00 
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
                printf("Link Close with unrecognized reason %x\n", reason);
            } else {
                printf("Link Close with reason %x\n", reason);
            }
            break;
        default:
            printf("BearerOpcode %x reserved for future use\n", bearer_opcode);
            break;
    }

}

static void pb_adv_send_pdu(void){
    pbv_adv_send_msg = 1;
    prov_msg_out_pos = 0;
    adv_bearer_request_can_send_now_for_pb_adv();
}

static void pbv_adv_pdu_complete(void){
    // Verify FCS
    uint8_t pdu_crc = btstack_crc8_calc(prov_msg_buffer, prov_msg_len);
    if (pdu_crc != prov_msg_fcs){
        printf("Incoming PDU: fcs %02x, calculated %02x -> drop packet\n", prov_msg_fcs, btstack_crc8_calc(prov_msg_buffer, prov_msg_len));
        return;
    }

    // Ack Transaction
    pbv_adv_send_ack = 1;
    adv_bearer_request_can_send_now_for_pb_adv();

    // Forward to Provisioning
    pbv_adv_packet_handler();
}

static void pbv_adv_handle_transaction_start(uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){

    // check if previous transation incomplete
    if (prov_msg_len){
        printf("previous transaction %x not completed, msg pos %u, len %u\n", pbv_adv_transaction_nr_incoming, prov_msg_pos, prov_msg_len);
        return;
    }
    if (transaction_nr == pbv_adv_transaction_nr_incoming){
        printf("packet from previous transaction %x\n", transaction_nr);
        return;
    }
    pbv_adv_transaction_nr_incoming = transaction_nr;

    // setup buffer
    prov_msg_len       = big_endian_read_16(pdu, 1);
    prov_msg_seg_total = pdu[0] >> 2;
    prov_msg_fcs       = pdu[3];

    uint16_t payload_len = size - 4;
    memcpy(prov_msg_buffer, &pdu[4], payload_len);
    prov_msg_pos = payload_len;

    // complete?
    if (prov_msg_seg_total == 0){
        if (prov_msg_pos == prov_msg_len){
            pbv_adv_pdu_complete();
        } else {
            // invalid msg len
            printf("invalid msg len %u, expected %u\n", prov_msg_pos, prov_msg_len);
        }
        prov_msg_len = 0;
        prov_msg_seg_next  = 0;
    } else {
        prov_msg_seg_next  = 1;
    }
}

static void pb_adv_handle_transaction_cont(uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
    // check segment index
    uint8_t seg = pdu[0] >> 2;
    if (seg != prov_msg_seg_next) {
        printf("wrong segment %u, expected %u\n", seg, prov_msg_seg_next);
        return;
    }
    // check transaction nr
    if (transaction_nr != pbv_adv_transaction_nr_incoming){
        printf("transaction nr %x but expected %x\n", transaction_nr, pbv_adv_transaction_nr_incoming);
        return;
    }
    uint16_t remaining_space = sizeof(prov_msg_buffer) - prov_msg_pos;
    uint16_t fragment_size   = size - 1;
    if (fragment_size > remaining_space) return;
    memcpy(&prov_msg_buffer[prov_msg_pos], &pdu[1], fragment_size);
    prov_msg_pos += fragment_size;

     // last segment
     if (prov_msg_seg_total == seg){
        if (prov_msg_pos == prov_msg_len){
            pbv_adv_pdu_complete();
        } else {
            // invalid msg len
            printf("invalid msg len %u, expected %u\n", prov_msg_pos, prov_msg_len);
        }
        prov_msg_len = 0;
        prov_msg_seg_next = 0;
     } else {
        prov_msg_seg_next++;
     }
}

static void pbv_adv_handle_transaction_ack(uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
    if (transaction_nr == pbv_adv_transaction_nr_outgoing){
        pbv_adv_transaction_nr_outgoing++;
        if (pbv_adv_transaction_nr_outgoing == 0x00){
            pbv_adv_transaction_nr_outgoing = 0x80;
        }
        printf("Transaction ACK %x received\n", transaction_nr);
    } else {
        printf("Unexpected Transaction ACK for %x\n", transaction_nr);
    }
}

// static void provisioning_run(void){
// }

static void pb_adv_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    if (packet_type != HCI_EVENT_PACKET) return;
    const uint8_t * data;
    uint8_t  length;
    uint32_t link_id;
    uint8_t  transaction_nr;
    uint8_t  generic_provisioning_control;
    switch(packet[0]){
        case GAP_EVENT_ADVERTISING_REPORT:
            data = gap_event_advertising_report_get_data(packet);
            // PDB ADV PDU
            length = data[0];
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
            printf("\npbv_adv: trans %x - ", transaction_nr);
            printf_hexdump(&data[7], length-6);

            switch (generic_provisioning_control_format){
                case MESH_GPCF_TRANSACTION_START:
                    pbv_adv_handle_transaction_start(transaction_nr, &data[7], length-6);
                    break;
                case MESH_GPCF_TRANSACTION_CONT:
                    pb_adv_handle_transaction_cont(transaction_nr, &data[7], length-6);
                    break;
                case MESH_GPCF_TRANSACTION_ACK:
                    pbv_adv_handle_transaction_ack(transaction_nr, &data[7], length-6);
                    break;
                case MESH_GPCF_PROV_BEARER_CONTROL:
                    pbv_adv_handle_bearer_control(link_id, transaction_nr, &data[7], length-6);
                    break;
            }
            break;
        case HCI_EVENT_MESH_META:
            switch(packet[2]){
                case MESH_SUBEVENT_CAN_SEND_NOW:
                    if (link_state == LINK_STATE_W2_SEND_ACK){
                        link_state = LINK_STATE_OPEN;
                        pbv_adv_transaction_nr_outgoing = 0x80;
                        // build packet
                        uint8_t buffer[6];
                        big_endian_store_32(buffer, 0, pbv_adv_link_id);
                        buffer[4] = 0;
                        buffer[5] = (1 << 2) | 3; // Link Ack | Provisioning Bearer Control
                        adv_bearer_send_pb_adv(buffer, sizeof(buffer));
                        log_info("link ack %08x", pbv_adv_link_id);
                        printf("Sending Link Ack\n");
                        break;
                    }
                    if (pbv_adv_send_ack){
                        pbv_adv_send_ack = 0;
                        uint8_t buffer[6];
                        big_endian_store_32(buffer, 0, pbv_adv_link_id);
                        buffer[4] = pbv_adv_transaction_nr_incoming;
                        buffer[5] = MESH_GPCF_TRANSACTION_ACK;
                        adv_bearer_send_pb_adv(buffer, sizeof(buffer));
                        log_info("transaction ack %08x", pbv_adv_link_id);
                        printf("Sending Transaction Ack (%x)\n", pbv_adv_transaction_nr_incoming);

                        if (prov_msg_out_len){
                            adv_bearer_request_can_send_now_for_pb_adv();
                        }
                        break;
                    }
                    if (pbv_adv_send_msg){
                        uint8_t buffer[29]; // ADV MTU
                        big_endian_store_32(buffer, 0, pbv_adv_link_id);
                        buffer[4] = pbv_adv_transaction_nr_outgoing;
                        uint16_t bytes_left;
                        uint16_t pos;
                        if (prov_msg_out_pos == 0){
                            // Transaction start
                            int seg_n = prov_msg_out_len / 24;
                            prov_msg_out_seg = 0;
                            buffer[5] = seg_n << 2 | MESH_GPCF_TRANSACTION_START;
                            big_endian_store_16(buffer, 6, prov_msg_out_len);
                            buffer[8] = btstack_crc8_calc(prov_msg_buffer, prov_msg_out_len);
                            pos = 9;
                            bytes_left = 24 - 4;
                            printf("Sending Provisioning Start (trans %x): ", pbv_adv_transaction_nr_outgoing);
                        } else {
                            // Transaction continue
                            buffer[5] = prov_msg_out_seg << 2 | MESH_GPCF_TRANSACTION_CONT;
                            pos = 6;
                            bytes_left = 24 - 1;
                            printf("Sending Provisioning Cont  (trans %x): ", pbv_adv_transaction_nr_outgoing);
                        }
                        prov_msg_out_seg++;
                        uint16_t bytes_to_copy = btstack_min(bytes_left, prov_msg_out_len - prov_msg_out_pos);
                        memcpy(&buffer[pos], &prov_msg_buffer[prov_msg_out_pos], bytes_to_copy);
                        pos += bytes_to_copy;
                        prov_msg_out_pos += bytes_to_copy;
                        printf("bytes %u, pos %u, len %u: ", bytes_to_copy, prov_msg_out_pos, prov_msg_out_len);
                        printf_hexdump(buffer, pos);

                        if (prov_msg_out_pos == prov_msg_out_len){
                            // done
                            pbv_adv_send_msg = 0;
                            prov_msg_out_len = 0;
                        }
                        adv_bearer_send_pb_adv(buffer, pos);
                        log_info("Prov msg");

                        if (prov_msg_out_len){
                            adv_bearer_request_can_send_now_for_pb_adv();
                        }
                        break;
                    }
                    break;
                default:
                    break;
            }
        default:
            break;
    }
}

// Provisioning Bearer Control

#define MESH_PROV_INVITE            0x00
#define MESH_PROV_CAPABILITIES      0x01
#define MESH_PROV_START             0x02
#define MESH_PROV_PUB_KEY           0x03
#define MESH_PROV_INPUT_COMPLETE    0x04
#define MESH_PROV_CONFIRM           0x05
#define MESH_PROV_RANDOM            0x06
#define MESH_PROV_DATA              0x07
#define MESH_PROV_COMPLETE          0x08
#define MESH_PROV_FAILED            0x09

#define MESH_OUTPUT_OOB_BLINK       0x01
#define MESH_OUTPUT_OOB_BEEP        0x02
#define MESH_OUTPUT_OOB_VIBRATE     0x04
#define MESH_OUTPUT_OOB_NUMBER      0x08
#define MESH_OUTPUT_OOB_STRING      0x10

#define MESH_INPUT_OOB_PUSH         0x01
#define MESH_INPUT_OOB_TWIST        0x02
#define MESH_INPUT_OOB_NUMBER       0x04
#define MESH_INPUT_OOB_STRING       0x08


#if 0
typedef enum {
    PROV_STATE_W4_INVITE,
    PROV_STATE_W2_SEND_CAPABILITIES,
} prov_state_t;
static prov_state_t prov_state;
#endif



static void provisioning_handle_invite(void){
    // handle invite message
    // TODO: store Attention Timer State

    // setup response 
    prov_msg_out_len = 12;

    prov_msg_buffer[0] = MESH_PROV_CAPABILITIES;

    // TOOD: get actual number
    /* Number of Elements supported */
    prov_msg_buffer[1] = 1;

    /* Supported algorithms - FIPS P-256 Eliptic Curve */
    big_endian_store_16(prov_msg_buffer, 2, 1);

    /* Public Key Type - Public Key OOB information available */
    prov_msg_buffer[4] = 0;

    /* Static OOB Type - Static OOB information available */
    prov_msg_buffer[5] = 1; 

    /* Output OOB Size - max of 8 */
    prov_msg_buffer[6] = 8; 

    /* Output OOB Action */
    big_endian_store_16(prov_msg_buffer, 7, MESH_OUTPUT_OOB_NUMBER | MESH_OUTPUT_OOB_STRING);

    /* Input OOB Size - max of 8*/
    prov_msg_buffer[9] = 8; 

    /* Input OOB Action */
    big_endian_store_16(prov_msg_buffer, 10, MESH_INPUT_OOB_STRING | MESH_OUTPUT_OOB_NUMBER);

    // send
    pb_adv_send_pdu();
}

static void provisioning_handle_public_key(void){

    // setup response 
    prov_msg_out_len = 65;

    // TODO: use real public key
    memset(prov_msg_buffer, 0, 65);
    prov_msg_buffer[0] = MESH_PROV_PUB_KEY;

    // send
    pb_adv_send_pdu();
}

static void provisioning_handle_pdu(void){

    uint8_t type = prov_msg_buffer[0];
    printf("Prov MSG Type %x\n", type);
    printf_hexdump(prov_msg_buffer, prov_msg_len);

    // dispatch msg
    switch (type){
        case MESH_PROV_INVITE:
            printf("MESH_PROV_INVITE\n");
            provisioning_handle_invite();
            break;
        case MESH_PROV_PUB_KEY:
            printf("MESH_PROV_PUB_KEY\n");
            provisioning_handle_public_key();
            break;            
        default:
            printf("TODO: handle provisioning msg type %x\n", type);
            break;
    }
}

static void provisioning_init(void){
    pbv_adv_register_handler(&provisioning_handle_pdu);
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

    provisioning_init();

    // turn on!
	hci_power_control(HCI_POWER_ON);
	    
    return 0;
}
/* EXAMPLE_END */
