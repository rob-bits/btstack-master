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

#define __BTSTACK_FILE__ "pb_adv.c"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ble/mesh/adv_bearer.h"
#include "ble/mesh/beacon.h"
#include "btstack_util.h"
#include "btstack_debug.h"
#include "btstack_event.h"

static void pb_adv_run(void);

/* taps: 32 31 29 1; characteristic polynomial: x^32 + x^31 + x^29 + x + 1 */
#define LFSR(a) ((a >> 1) ^ (uint32_t)((0 - (a & 1u)) & 0xd0000001u))

// PB-ADV - Provisioning Bearer using Advertisement Bearer

#define MESH_PROV_LINK_OPEN              0x00
#define MESH_PROV_LINK_ACK               0x01
#define MESH_PROV_LINK_CLOSE             0x02

typedef enum mesh_gpcf_format {
    MESH_GPCF_TRANSACTION_START = 0,
    MESH_GPCF_TRANSACTION_ACK,
    MESH_GPCF_TRANSACTION_CONT,
    MESH_GPCF_PROV_BEARER_CONTROL,
} mesh_gpcf_format_t;

typedef enum {
    LINK_STATE_W4_OPEN,
    LINK_STATE_W2_SEND_ACK,
    LINK_STATE_OPEN,
} link_state_t;
static link_state_t link_state;

static const uint8_t * pb_adv_device_uuid;

static uint8_t  pb_adv_msg_in_buffer[100];   // TODO: how large are prov messages?

static uint32_t pb_adv_link_id;
static uint8_t  pb_adv_transaction_nr_incoming;
static uint8_t  pb_adv_transaction_nr_outgoing;
static uint8_t  pb_adv_send_ack;
static uint8_t  pb_adv_send_msg;

static uint8_t  pb_adv_msg_in_seg_total; // total segments
static uint8_t  pb_adv_msg_in_seg_next;  // next expected segment 
static uint16_t pb_adv_msg_in_len;   // 
static uint16_t pb_adv_msg_in_pos;
static uint8_t  pb_adv_msg_in_fcs;

static uint16_t        pb_adv_msg_out_len;
static uint16_t        pb_adv_msg_out_pos;
static uint8_t         pb_adv_msg_out_seg;
static const uint8_t * pb_adv_msg_out_buffer;

static uint32_t pb_adv_lfsr;

static uint8_t                pb_adv_random_delay_active;
static btstack_timer_source_t pb_adv_random_delay_timer;

static btstack_packet_handler_t pb_adv_packet_handler;

// poor man's random number generator
static uint32_t pb_adv_random(void){
    pb_adv_lfsr = LFSR(pb_adv_lfsr);
    return pb_adv_lfsr;
}

static void pb_adv_handle_bearer_control(uint32_t link_id, uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
    uint8_t bearer_opcode = pdu[0] >> 2;
    uint8_t reason;
    switch (bearer_opcode){
        case MESH_PROV_LINK_OPEN: // Open a session on a bearer with a device
            // does it match our device_uuid?
            if (memcmp(&pdu[1], pb_adv_device_uuid, 16) != 0) break;
            switch(link_state){
                case LINK_STATE_W4_OPEN:
                    pb_adv_link_id = link_id;
                    pb_adv_transaction_nr_incoming = 0xff;  // first transaction nr will be 0x00 
                    log_info("link open, id %08x", pb_adv_link_id);
                    printf("Link Open - id %08x. we're the unprovisioned device\n", pb_adv_link_id);
                    link_state = LINK_STATE_W2_SEND_ACK;
                    adv_bearer_request_can_send_now_for_pb_adv();
                    break;
                case LINK_STATE_W2_SEND_ACK:
                    break;
                case LINK_STATE_OPEN:
                    if (pb_adv_link_id != link_id) break;
                    log_info("link open, resend ACK");
                    link_state = LINK_STATE_W2_SEND_ACK;
                    adv_bearer_request_can_send_now_for_pb_adv();
                    break;
            }
            break;
        case MESH_PROV_LINK_ACK:   // Acknowledge a session on a bearer
            break;
        case MESH_PROV_LINK_CLOSE: // Close a session on a bearer
            // does it match link id
            if (link_id != pb_adv_link_id) break;
            reason = pdu[1];
            link_state = LINK_STATE_W4_OPEN;
            log_info("link close, reason %x", reason);
            break;
        default:
            log_info("BearerOpcode %x reserved for future use\n", bearer_opcode);
            break;
    }

}

static void pb_adv_pdu_complete(void){
    // Verify FCS
    uint8_t pdu_crc = btstack_crc8_calc((uint8_t*)pb_adv_msg_in_buffer, pb_adv_msg_in_len);
    if (pdu_crc != pb_adv_msg_in_fcs){
        printf("Incoming PDU: fcs %02x, calculated %02x -> drop packet\n", pb_adv_msg_in_fcs, btstack_crc8_calc(pb_adv_msg_in_buffer, pb_adv_msg_in_len));
        return;
    }

    // Ack Transaction
    pb_adv_send_ack = 1;
    pb_adv_run();

    // Forward to Provisioning
    pb_adv_packet_handler(PROVISIONING_DATA_PACKET, 0, pb_adv_msg_in_buffer, pb_adv_msg_in_len);
}

static void pb_adv_handle_transaction_start(uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){

    // check if previous transation incomplete
    if (pb_adv_msg_in_len){
        printf("previous transaction %x not completed, msg pos %u, len %u\n", pb_adv_transaction_nr_incoming, pb_adv_msg_in_pos, pb_adv_msg_in_len);
        return;
    }
    if (transaction_nr == pb_adv_transaction_nr_incoming){
        printf("packet from previous transaction %x\n", transaction_nr);
        return;
    }
    pb_adv_transaction_nr_incoming = transaction_nr;

    // setup buffer
    pb_adv_msg_in_len       = big_endian_read_16(pdu, 1);
    pb_adv_msg_in_seg_total = pdu[0] >> 2;
    pb_adv_msg_in_fcs       = pdu[3];

    uint16_t payload_len = size - 4;
    memcpy(pb_adv_msg_in_buffer, &pdu[4], payload_len);
    pb_adv_msg_in_pos = payload_len;

    // complete?
    if (pb_adv_msg_in_seg_total == 0){
        if (pb_adv_msg_in_pos == pb_adv_msg_in_len){
            pb_adv_pdu_complete();
        } else {
            // invalid msg len
            printf("invalid msg len %u, expected %u\n", pb_adv_msg_in_pos, pb_adv_msg_in_len);
        }
        pb_adv_msg_in_len = 0;
        pb_adv_msg_in_seg_next  = 0;
    } else {
        pb_adv_msg_in_seg_next  = 1;
    }
}

static void pb_adv_handle_transaction_cont(uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
    // check segment index
    uint8_t seg = pdu[0] >> 2;
    if (seg != pb_adv_msg_in_seg_next) {
        printf("wrong segment %u, expected %u\n", seg, pb_adv_msg_in_seg_next);
        return;
    }
    // check transaction nr
    if (transaction_nr != pb_adv_transaction_nr_incoming){
        printf("transaction nr %x but expected %x\n", transaction_nr, pb_adv_transaction_nr_incoming);
        return;
    }
    uint16_t remaining_space = sizeof(pb_adv_msg_in_buffer) - pb_adv_msg_in_pos;
    uint16_t fragment_size   = size - 1;
    if (fragment_size > remaining_space) return;
    memcpy(&pb_adv_msg_in_buffer[pb_adv_msg_in_pos], &pdu[1], fragment_size);
    pb_adv_msg_in_pos += fragment_size;

     // last segment
     if (pb_adv_msg_in_seg_total == seg){
        if (pb_adv_msg_in_pos == pb_adv_msg_in_len){
            pb_adv_pdu_complete();
        } else {
            // invalid msg len
            printf("invalid msg len %u, expected %u\n", pb_adv_msg_in_pos, pb_adv_msg_in_len);
        }
        pb_adv_msg_in_len = 0;
        pb_adv_msg_in_seg_next = 0;
     } else {
        pb_adv_msg_in_seg_next++;
     }
}

static void pb_adv_handle_transaction_ack(uint8_t transaction_nr, const uint8_t * pdu, uint16_t size){
    if (transaction_nr == pb_adv_transaction_nr_outgoing){
        pb_adv_transaction_nr_outgoing++;
        if (pb_adv_transaction_nr_outgoing == 0x00){
            pb_adv_transaction_nr_outgoing = 0x80;
        }
        printf("Transaction ACK %x received\n", transaction_nr);
    } else {
        printf("Unexpected Transaction ACK for %x\n", transaction_nr);
    }
}

static int pbv_adv_packet_to_send(void){
    return pb_adv_send_ack || pb_adv_send_msg;
}

static void pbv_adv_timer_handler(btstack_timer_source_t * ts){
    pb_adv_random_delay_active = 0;
    if (!pbv_adv_packet_to_send()) return;
    adv_bearer_request_can_send_now_for_pb_adv();  
}

static void pb_adv_run(void){
    if (!pbv_adv_packet_to_send()) return;
    if (pb_adv_random_delay_active) return;

    // spec recommends 20-50 ms, we use 20-51 ms
    pb_adv_random_delay_active = 1;
    uint16_t random_delay_ms = 20 + (pb_adv_random() & 0x1f);
    log_info("random delay %u ms", random_delay_ms);
    printf("random delay %u ms\n", random_delay_ms);
    btstack_run_loop_set_timer_handler(&pb_adv_random_delay_timer, &pbv_adv_timer_handler);
    btstack_run_loop_set_timer(&pb_adv_random_delay_timer, random_delay_ms);
    btstack_run_loop_add_timer(&pb_adv_random_delay_timer);
}

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
                if (link_id    != pb_adv_link_id) break;
                if (link_state != LINK_STATE_OPEN) break;
            }
            printf("\npbv_adv: trans %x - ", transaction_nr);
            printf_hexdump(&data[7], length-6);

            switch (generic_provisioning_control_format){
                case MESH_GPCF_TRANSACTION_START:
                    pb_adv_handle_transaction_start(transaction_nr, &data[7], length-6);
                    break;
                case MESH_GPCF_TRANSACTION_CONT:
                    pb_adv_handle_transaction_cont(transaction_nr, &data[7], length-6);
                    break;
                case MESH_GPCF_TRANSACTION_ACK:
                    pb_adv_handle_transaction_ack(transaction_nr, &data[7], length-6);
                    break;
                case MESH_GPCF_PROV_BEARER_CONTROL:
                    pb_adv_handle_bearer_control(link_id, transaction_nr, &data[7], length-6);
                    break;
            }
            break;
        case HCI_EVENT_MESH_META:
            switch(packet[2]){
                case MESH_SUBEVENT_CAN_SEND_NOW:
                    if (link_state == LINK_STATE_W2_SEND_ACK){
                        link_state = LINK_STATE_OPEN;
                        pb_adv_transaction_nr_outgoing = 0x80;
                        // build packet
                        uint8_t buffer[6];
                        big_endian_store_32(buffer, 0, pb_adv_link_id);
                        buffer[4] = 0;
                        buffer[5] = (1 << 2) | 3; // Link Ack | Provisioning Bearer Control
                        adv_bearer_send_pb_adv(buffer, sizeof(buffer));
                        log_info("link ack %08x", pb_adv_link_id);
                        printf("Sending Link Ack\n");
                        break;
                    }
                    if (pb_adv_send_ack){
                        pb_adv_send_ack = 0;
                        uint8_t buffer[6];
                        big_endian_store_32(buffer, 0, pb_adv_link_id);
                        buffer[4] = pb_adv_transaction_nr_incoming;
                        buffer[5] = MESH_GPCF_TRANSACTION_ACK;
                        adv_bearer_send_pb_adv(buffer, sizeof(buffer));
                        log_info("transaction ack %08x", pb_adv_link_id);
                        printf("Sending Transaction Ack (%x)\n", pb_adv_transaction_nr_incoming);
                        pb_adv_run();
                        break;
                    }
                    if (pb_adv_send_msg){
                        if (pb_adv_msg_out_pos == pb_adv_msg_out_len){
                            pb_adv_send_msg = 0;
                            printf("should not get here..\n");
                            break;
                        }
                        uint8_t buffer[29]; // ADV MTU
                        big_endian_store_32(buffer, 0, pb_adv_link_id);
                        buffer[4] = pb_adv_transaction_nr_outgoing;
                        uint16_t bytes_left;
                        uint16_t pos;
                        if (pb_adv_msg_out_pos == 0){
                            // Transaction start
                            int seg_n = pb_adv_msg_out_len / 24;
                            pb_adv_msg_out_seg = 0;
                            buffer[5] = seg_n << 2 | MESH_GPCF_TRANSACTION_START;
                            big_endian_store_16(buffer, 6, pb_adv_msg_out_len);
                            buffer[8] = btstack_crc8_calc((uint8_t*)pb_adv_msg_out_buffer, pb_adv_msg_out_len);
                            pos = 9;
                            bytes_left = 24 - 4;
                            printf("Sending Start (trans %x): ", pb_adv_transaction_nr_outgoing);
                        } else {
                            // Transaction continue
                            buffer[5] = pb_adv_msg_out_seg << 2 | MESH_GPCF_TRANSACTION_CONT;
                            pos = 6;
                            bytes_left = 24 - 1;
                            printf("Sending Cont  (trans %x): ", pb_adv_transaction_nr_outgoing);
                        }
                        pb_adv_msg_out_seg++;
                        uint16_t bytes_to_copy = btstack_min(bytes_left, pb_adv_msg_out_len - pb_adv_msg_out_pos);
                        memcpy(&buffer[pos], &pb_adv_msg_out_buffer[pb_adv_msg_out_pos], bytes_to_copy);
                        pos += bytes_to_copy;
                        printf("bytes %u, pos %u, len %u: ", bytes_to_copy, pb_adv_msg_out_pos, pb_adv_msg_out_len);
                        printf_hexdump(buffer, pos);
                        pb_adv_msg_out_pos += bytes_to_copy;

                        if (pb_adv_msg_out_pos == pb_adv_msg_out_len){
                            // done
                            printf("Sending Done  (trans %x)\n", pb_adv_transaction_nr_outgoing);
                            pb_adv_send_msg = 0;
                            pb_adv_msg_out_len = 0;
                        }
                        adv_bearer_send_pb_adv(buffer, pos);
                        pb_adv_run();
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

void pb_adv_init(uint8_t * device_uuid){
    pb_adv_device_uuid = device_uuid;
    adv_bearer_register_for_pb_adv(&pb_adv_handler);
    pb_adv_lfsr = little_endian_read_32(device_uuid, 0);
    pb_adv_random();
}

void pb_adv_register_packet_handler(btstack_packet_handler_t packet_handler){
    pb_adv_packet_handler = packet_handler;
}

void pb_adv_send_pdu(const uint8_t * pdu, uint16_t size){
    pb_adv_msg_out_buffer = pdu;
    pb_adv_msg_out_len    = size;
    pb_adv_msg_out_pos = 0;
    pb_adv_send_msg = 1;
    pb_adv_run();
}
