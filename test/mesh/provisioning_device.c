/*
 * Copyright (C) 2017 BlueKitchen GmbH
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

#define __BTSTACK_FILE__ "provisioning_device.c"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ble/mesh/pb_adv.h"
#include "classic/rfcomm.h" // for crc8
#include "btstack.h"

#define PROVISIONING_PROTOCOL_TIMEOUT_MS 60000

// remote ecc
static uint8_t remote_ec_q[64];
static uint8_t dhkey[32];

// mesh k1 - might get moved to btstack_crypto and all vars go into btstack_crypto_mesh_k1_t struct
static uint8_t         mesh_k1_temp[16];
static void (*         mesh_k1_callback)(void * arg);
static void *          mesh_k1_arg;
static const uint8_t * mesh_k1_p;
static uint16_t        mesh_k1_p_len;
static uint8_t *       mesh_k1_result;

static void mesh_k1_temp_calculated(void * arg){
    btstack_crypto_aes128_cmac_t * request = (btstack_crypto_aes128_cmac_t*) arg;
    btstack_crypto_aes128_cmac_message(request, mesh_k1_temp, mesh_k1_p_len, mesh_k1_p, mesh_k1_result, mesh_k1_callback, mesh_k1_arg);
}

static void mesh_k1(btstack_crypto_aes128_cmac_t * request, const uint8_t * n, uint16_t n_len, const uint8_t * salt,
    const uint8_t * p, const uint16_t p_len, uint8_t * result, void (* callback)(void * arg), void * callback_arg){
    mesh_k1_callback = callback;
    mesh_k1_arg      = callback_arg;
    mesh_k1_p        = p;
    mesh_k1_p_len    = p_len;
    mesh_k1_result   = result;
    btstack_crypto_aes128_cmac_message(request, salt, n_len, n, mesh_k1_temp, mesh_k1_temp_calculated, request);
}

// mesh k3 - might get moved to btstack_crypto and all vars go into btstack_crypto_mesh_k3_t struct
static const uint8_t   mesh_k3_tag[5] = { 'i', 'd', '6', '4', 0x01}; 
static uint8_t         mesh_k3_salt[16];
static uint8_t         mesh_k3_temp[16];
static uint8_t         mesh_k3_result128[16];
static void (*         mesh_k3_callback)(void * arg);
static void *          mesh_k3_arg;
static const uint8_t * mesh_k3_n;
static uint8_t       * mesh_k3_result;

static void mesh_k3_result128_calculated(void * arg){
    UNUSED(arg);
    memcpy(mesh_k3_result, mesh_k3_result128, 8);
    (*mesh_k3_callback)(mesh_k3_arg);        
}
static void mesh_k3_temp_callback(void * arg){
    btstack_crypto_aes128_cmac_t * request = (btstack_crypto_aes128_cmac_t*) arg;
    btstack_crypto_aes128_cmac_message(request, mesh_k3_temp, sizeof(mesh_k3_tag), mesh_k3_tag, mesh_k3_result128, mesh_k3_result128_calculated, request);
}
static void mesh_k3_salt_calculated(void * arg){
    btstack_crypto_aes128_cmac_t * request = (btstack_crypto_aes128_cmac_t*) arg;
    btstack_crypto_aes128_cmac_message(request, mesh_k3_salt, 16, mesh_k3_n, mesh_k3_temp, mesh_k3_temp_callback, request);
}
static void mesh_k3(btstack_crypto_aes128_cmac_t * request, const uint8_t * n, uint8_t * result, void (* callback)(void * arg), void * callback_arg){
    mesh_k3_callback = callback;
    mesh_k3_arg      = callback_arg;
    mesh_k3_n        = n;
    mesh_k3_result   = result;
    btstack_crypto_aes128_cmac_zero(request, 4, (const uint8_t *) "smk3", mesh_k3_salt, mesh_k3_salt_calculated, request);
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
// virtual commands
#define MESH_PROV_USER_INPUT_OOB    0xF0
#define MESH_PROV_INVALID           0xFF

#define MESH_OUTPUT_OOB_BLINK       0x01
#define MESH_OUTPUT_OOB_BEEP        0x02
#define MESH_OUTPUT_OOB_VIBRATE     0x04
#define MESH_OUTPUT_OOB_NUMBER      0x08
#define MESH_OUTPUT_OOB_STRING      0x10

#define MESH_INPUT_OOB_PUSH         0x01
#define MESH_INPUT_OOB_TWIST        0x02
#define MESH_INPUT_OOB_NUMBER       0x04
#define MESH_INPUT_OOB_STRING       0x08

static btstack_packet_handler_t prov_packet_handler;

static uint8_t  prov_next_command;

static uint8_t  prov_buffer_out[100];   // TODO: how large are prov messages?
// ConfirmationInputs = ProvisioningInvitePDUValue || ProvisioningCapabilitiesPDUValue || ProvisioningStartPDUValue || PublicKeyProvisioner || PublicKeyDevice
static uint8_t  prov_confirmation_inputs[1 + 11 + 5 + 64 + 64];
static uint8_t  prov_authentication_action;
static uint8_t  prov_public_key_oob_used;
static uint8_t  prov_emit_public_key_oob_active;
static uint8_t  prov_emit_output_oob_active;
static uint8_t  prov_ec_q[64];

static const uint8_t * prov_public_key_oob_q;
static const uint8_t * prov_public_key_oob_d;

// num elements
static uint8_t  prov_num_elements = 1;

// capabilites
static const uint8_t * prov_static_oob_data;

static uint16_t  prov_static_oob_len;
static uint16_t  prov_output_oob_actions;
static uint16_t  prov_input_oob_actions;
static uint8_t   prov_public_key_oob_available;
static uint8_t   prov_static_oob_available;
static uint8_t   prov_output_oob_size;
static uint8_t   prov_input_oob_size;
static uint8_t   prov_error_code;
static uint8_t   prov_message_to_send;
static uint8_t   prov_waiting_for_outgoing_complete;

#ifdef ENABLE_ATTENTION_TIMER
static btstack_timer_source_t       prov_attention_timer;
#endif

static btstack_timer_source_t       prov_protocol_timer;

static btstack_crypto_aes128_cmac_t prov_cmac_request;
static btstack_crypto_random_t      prov_random_request;
static btstack_crypto_ecc_p256_t    prov_ecc_p256_request;
static btstack_crypto_ccm_t         prov_ccm_request;

// ConfirmationDevice
static uint8_t confirmation_device[16];
// ConfirmationSalt
static uint8_t confirmation_salt[16];
// ConfirmationKey
static uint8_t confirmation_key[16];
// RandomDevice
static uint8_t random_device[16];
// ProvisioningSalt
static uint8_t provisioning_salt[16];
// AuthValue
static uint8_t auth_value[16];
// SessionKey
static uint8_t session_key[16];
// SessionNonce
static uint8_t session_nonce[16];
// EncProvisioningData
static uint8_t enc_provisioning_data[25];
// ProvisioningData
static uint8_t provisioning_data[25];
// DeviceKey
static uint8_t device_key[16];
// NetKey
static uint8_t  net_key[16];
// NetKeyIndex
static uint16_t net_key_index;

static uint8_t  flags;

static uint32_t iv_index;
static uint16_t unicast_address;

static const uint8_t id128_tag[] = { 'i', 'd', '1', '2', '8', 0x01};

// derived
static uint8_t network_id[8];
static uint8_t beacon_key[16];

static void dump_data(uint8_t * buffer, uint16_t size){
    static int data_counter = 1;
    char var_name[80];
    sprintf(var_name, "test_data_%02u", data_counter);
    printf("uint8_t %s[] = { ", var_name);
    for (int i = 0; i < size ; i++){
        if ((i % 16) == 0) printf("\n    ");
        printf ("0x%02x, ", buffer[i]);
    }
    printf("};\n");
    data_counter++;
}

static void provisioning_emit_event(uint8_t mesh_subevent, uint16_t pb_adv_cid){
    if (!prov_packet_handler) return;
    uint8_t event[5] = { HCI_EVENT_MESH_META, 3, mesh_subevent};
    little_endian_store_16(event, 3, pb_adv_cid);
    prov_packet_handler(HCI_EVENT_PACKET, 0, event, sizeof(event));
}

static void provisioning_emit_output_oob_event(uint16_t pb_adv_cid, uint32_t number){
    if (!prov_packet_handler) return;
    uint8_t event[9] = { HCI_EVENT_MESH_META, 7, MESH_PB_PROV_START_EMIT_OUTPUT_OOB};
    little_endian_store_16(event, 3, pb_adv_cid);
    little_endian_store_16(event, 5, number);
    prov_packet_handler(HCI_EVENT_PACKET, 0, event, sizeof(event));
}

static void provisiong_timer_handler(btstack_timer_source_t * ts){
    UNUSED(ts);
    printf("Provisioning Protocol Timeout -> Close Link!\n");
    // TODO: use actual pb_adv_cid
    pb_adv_close_link(1, 1);
}

// The provisioning protocol shall have a minimum timeout of 60 seconds that is reset
// each time a provisioning protocol PDU is sent or received
static void provisioning_timer_start(void){
    btstack_run_loop_remove_timer(&prov_protocol_timer);
    btstack_run_loop_set_timer_handler(&prov_protocol_timer, &provisiong_timer_handler);
    btstack_run_loop_set_timer(&prov_protocol_timer, PROVISIONING_PROTOCOL_TIMEOUT_MS);
    btstack_run_loop_add_timer(&prov_protocol_timer);
}

static void provisioning_timer_stop(void){
    btstack_run_loop_remove_timer(&prov_protocol_timer);
}

static void provisioning_attention_timer_timeout(btstack_timer_source_t * ts){
    UNUSED(ts);
#ifdef ENABLE_ATTENTION_TIMER
    // TODO: check if provisioning complete, stop attention
#endif
}

// Outgoing Provisioning PDUs

static void provisioning_send_provisioning_error(void){
    // setup response 
    prov_buffer_out[0] = MESH_PROV_FAILED;
    prov_buffer_out[1] = prov_error_code;
    pb_adv_send_pdu(prov_buffer_out, 2);
}

static void provisioning_send_capabilites(void){
    // setup response 
    prov_buffer_out[0] = MESH_PROV_CAPABILITIES;

    /* Number of Elements supported */
    prov_buffer_out[1] = prov_num_elements;

    /* Supported algorithms - FIPS P-256 Eliptic Curve */
    big_endian_store_16(prov_buffer_out, 2, 1);

    /* Public Key Type - Public Key OOB information available */
    prov_buffer_out[4] = prov_public_key_oob_available;

    /* Static OOB Type - Static OOB information available */
    prov_buffer_out[5] = prov_static_oob_available; 

    /* Output OOB Size - max of 8 */
    prov_buffer_out[6] = prov_output_oob_size; 

    /* Output OOB Action */
    big_endian_store_16(prov_buffer_out, 7, prov_output_oob_actions);

    /* Input OOB Size - max of 8*/
    prov_buffer_out[9] = prov_input_oob_size; 

    /* Input OOB Action */
    big_endian_store_16(prov_buffer_out, 10, prov_input_oob_actions);

    // store for confirmation inputs: len 11
    memcpy(&prov_confirmation_inputs[1], &prov_buffer_out[1], 11);

    // send
    pb_adv_send_pdu(prov_buffer_out, 12);    
}

static void provisioning_send_public_key(void){
    // setup response 
    prov_buffer_out[0] = MESH_PROV_PUB_KEY;
    memcpy(&prov_buffer_out[1], prov_ec_q, 64);

    // store for confirmation inputs: len 64
    memcpy(&prov_confirmation_inputs[81], &prov_buffer_out[1], 64);

    // send
    pb_adv_send_pdu(prov_buffer_out, 65);
}

static void provisioning_send_input_complete(void){
    // setup response 
    prov_buffer_out[0] = MESH_PROV_INPUT_COMPLETE;

    // send
    pb_adv_send_pdu(prov_buffer_out, 17);
}
static void provisioning_send_confirm(void){
    // setup response 
    prov_buffer_out[0] = MESH_PROV_CONFIRM;
    memcpy(&prov_buffer_out[1], confirmation_device, 16);

    // send
    pb_adv_send_pdu(prov_buffer_out, 17);
}

static void provisioning_send_random(void){
    // setup response 
    prov_buffer_out[0] = MESH_PROV_RANDOM;
    memcpy(&prov_buffer_out[1],  random_device, 16);

    // send pdu
    pb_adv_send_pdu(prov_buffer_out, 17);
}

static void provisioning_send_complete(void){
    // setup response 
    prov_buffer_out[0] = MESH_PROV_COMPLETE;

    // send pdu
    pb_adv_send_pdu(prov_buffer_out, 1);
}

static void provisioning_send_pdu(void){
    int start_timer = 1;
    switch (prov_message_to_send){
        case MESH_PROV_FAILED:
            start_timer = 0;    // game over
            provisioning_send_provisioning_error();
            break;
        case MESH_PROV_CAPABILITIES:
            provisioning_send_capabilites();
            break;
        case MESH_PROV_PUB_KEY:
            provisioning_send_public_key();
            break;
        case MESH_PROV_INPUT_COMPLETE:
            provisioning_send_input_complete();
            break;
        case MESH_PROV_CONFIRM:
            provisioning_send_confirm();
            break;
        case MESH_PROV_RANDOM:
            provisioning_send_random();
            break;
        case MESH_PROV_COMPLETE:
            provisioning_send_complete();
            break;
        default:
            return;
    }
    if (start_timer){
        provisioning_timer_start();
    }
    printf("Send message 0x%0x\n", prov_message_to_send);
    prov_waiting_for_outgoing_complete = 1;
    prov_message_to_send = MESH_PROV_INVALID;
}

// End of outgoing PDUs

static void provisioning_queue_pdu(uint8_t message){
    printf("Queue message 0x%08x (outgoing active %u)\n", message, prov_waiting_for_outgoing_complete);
    prov_message_to_send = message;
    // wait for last msg ack
    if (prov_waiting_for_outgoing_complete) return;
    provisioning_send_pdu(); 
}

static void provisioning_done(void){
    if (prov_emit_public_key_oob_active){
        prov_emit_public_key_oob_active = 0;
        provisioning_emit_event(MESH_PB_PROV_STOP_EMIT_PUBLIC_KEY_OOB, 1);
    }
    if (prov_emit_output_oob_active){
        prov_emit_output_oob_active = 0;
        provisioning_emit_event(MESH_PB_PROV_STOP_EMIT_OUTPUT_OOB, 1);
    }
    prov_message_to_send = MESH_PROV_INVALID;
    prov_next_command    = MESH_PROV_INVITE;
}

static void provisioning_handle_provisioning_error(uint8_t error_code){
    provisioning_timer_stop();
    prov_error_code = error_code;
    provisioning_queue_pdu(MESH_PROV_FAILED);

    // TODO: use actual pb_adv_cid
    // pb_adv_close_link(1, 2);
    provisioning_done();
}

static void provisioning_handle_invite(uint8_t *packet, uint16_t size){

    if (size != 1) return;

    // store for confirmation inputs: len 1
    memcpy(&prov_confirmation_inputs[0], packet, 1);

    // handle invite message
#ifdef ENABLE_ATTENTION_TIMER
    uint32_t attention_timer_timeout_ms = packet[0] * 1000;
    btstack_run_loop_set_timer_handler(&prov_attention_timer, &provisioning_attention_timer_timeout);
    btstack_run_loop_set_timer(&prov_attention_timer, attention_timer_timeout_ms);
    btstack_run_loop_add_timer(&prov_attention_timer);
#endif

    // queue capabilities pdu
    provisioning_queue_pdu(MESH_PROV_CAPABILITIES);

    // next state
    prov_next_command = MESH_PROV_START;
}

static void provisioning_handle_start(uint8_t * packet, uint16_t size){

    if (size != 5) return;

    // validate Algorithm
    int ok = 1;
    if (packet[0] > 0x00){
        ok = 0;
    }
    // validate Publik Key
    if (packet[1] > 0x01){
        ok = 0;
    }
    // validate Authentication Method
    switch (packet[2]){
        case 0:
        case 1:
            if (packet[3] != 0 || packet[4] != 0){
                ok = 0;
                break;
            }
            break;
        case 2:
            if (packet[3] > 0x04 || packet[4] == 0 || packet[4] > 0x08){
                ok = 0;
                break;
            }
            break;
        case 3:
            if (packet[3] > 0x03 || packet[4] == 0 || packet[4] > 0x08){
                ok = 0;
                break;
            }
            break;
    }
    if (!ok){
        printf("PROV_START arguments incorrect\n");
        provisioning_handle_provisioning_error(0x02);
        return;
    }

    // store for confirmation inputs: len 5
    memcpy(&prov_confirmation_inputs[12], packet, 5);

    // public key oob
    prov_public_key_oob_used = packet[1];

    // output authentication action
    prov_authentication_action = packet[2];

    // start emit public OOK if specified
    if (prov_public_key_oob_available && prov_public_key_oob_used){
        provisioning_emit_event(MESH_PB_PROV_START_EMIT_PUBLIC_KEY_OOB, 1);
    }

    printf("PublicKey:  %02x\n", prov_public_key_oob_used);
    printf("AuthAction: %02x\n", prov_authentication_action);

    // next state
    prov_next_command = MESH_PROV_PUB_KEY;
}

static void provisioning_handle_auth_value_output_oob(void * arg){
    // limit auth value to single digit
    auth_value[15] = auth_value[15] % 9 + 1;

    printf("Output OOB: %u\n", auth_value[15]);

    // emit output oob value
    provisioning_emit_output_oob_event(1, auth_value[15]);
    prov_emit_output_oob_active = 1;

    // wait for confirm
    prov_next_command = MESH_PROV_CONFIRM;
}

static void provisioning_public_key_exchange_complete(void){

    // reset auth_value
    memset(auth_value, 0, sizeof(auth_value));

    // handle authentication method
    switch (prov_authentication_action){
        case 0x00:
            prov_next_command = MESH_PROV_CONFIRM;
            break;        
        case 0x01:
            memcpy(&auth_value[16-prov_static_oob_len], prov_static_oob_data, prov_static_oob_len);
            prov_next_command = MESH_PROV_CONFIRM;
            break;
        case 0x02:
            printf("Generate random for auth_value\n");
            // generate single byte of random data to use for authentication
            btstack_crypto_random_generate(&prov_random_request, &auth_value[15], 1, &provisioning_handle_auth_value_output_oob, NULL);
            break;
        case 0x03:
            // Input OOB
            printf("Input OOB requested\n");
            provisioning_emit_event(MESH_PB_PROV_INPUT_OOB_REQUEST, 1);
            // expect virtual command
            prov_next_command = MESH_PROV_USER_INPUT_OOB;
            break;
        default:
            break;
    }
}

static void provisioning_handle_public_key_dhkey(void * arg){
    UNUSED(arg);

    printf("DHKEY: ");
    printf_hexdump(dhkey, sizeof(dhkey));

    // skip sending own public key when public key oob is used
    if (prov_public_key_oob_available && prov_public_key_oob_used){
        // just copy key for confirmation inputs
        memcpy(&prov_confirmation_inputs[81], prov_ec_q, 64);
    } else {
        // queue public key pdu
        provisioning_queue_pdu(MESH_PROV_PUB_KEY);
    }

    provisioning_public_key_exchange_complete();
}

static void provisioning_handle_public_key(uint8_t *packet, uint16_t size){

    if (size != sizeof(remote_ec_q)) return;

    // stop emit public OOK if specified and send to crypto module
    if (prov_public_key_oob_available && prov_public_key_oob_used){
        provisioning_emit_event(MESH_PB_PROV_STOP_EMIT_PUBLIC_KEY_OOB, 1);

        printf("Replace generated ECC with Public Key OOB:");
        memcpy(prov_ec_q, prov_public_key_oob_q, 64);
        dump_data(prov_ec_q, sizeof(prov_ec_q));
        btstack_crypto_ecc_p256_set_key(prov_public_key_oob_q, prov_public_key_oob_d);
    }

    // store for confirmation inputs: len 64
    memcpy(&prov_confirmation_inputs[17], packet, 64);

    // store remote q
    memcpy(remote_ec_q, packet, sizeof(remote_ec_q));

    // calculate DHKey
    btstack_crypto_ecc_p256_calculate_dhkey(&prov_ecc_p256_request, remote_ec_q, dhkey, provisioning_handle_public_key_dhkey, NULL);
}

static void provisioning_handle_confirmation_device_calculated(void * arg){

    UNUSED(arg);

    printf("ConfirmationDevice: ");
    printf_hexdump(confirmation_device, sizeof(confirmation_device));

    // queue confirm pdu
    provisioning_queue_pdu(MESH_PROV_CONFIRM);

    // next state
    prov_next_command = MESH_PROV_RANDOM;
}

static void provisioning_handle_confirmation_random_device(void * arg){
    // re-use prov_confirmation_inputs buffer
    memcpy(&prov_confirmation_inputs[0],  random_device, 16);
    memcpy(&prov_confirmation_inputs[16], auth_value, 16);

    // calc confirmation device
    btstack_crypto_aes128_cmac_message(&prov_cmac_request, confirmation_key, 32, prov_confirmation_inputs, confirmation_device, &provisioning_handle_confirmation_device_calculated, NULL);
}

static void provisioning_handle_confirmation_k1_calculated(void * arg){
    printf("ConfirmationKey:   ");
    printf_hexdump(confirmation_key, sizeof(confirmation_key));

    printf("AuthValue: ");
    printf_hexdump(auth_value, 16);

    // generate random_device
    btstack_crypto_random_generate(&prov_random_request,random_device, 16, &provisioning_handle_confirmation_random_device, NULL);
}

static void provisioning_handle_confirmation_s1_calculated(void * arg){
    UNUSED(arg);

    // ConfirmationSalt
    printf("ConfirmationSalt:   ");
    printf_hexdump(confirmation_salt, sizeof(confirmation_salt));

    // ConfirmationKey
    mesh_k1(&prov_cmac_request, dhkey, sizeof(dhkey), confirmation_salt, (const uint8_t*) "prck", 4, confirmation_key, &provisioning_handle_confirmation_k1_calculated, NULL);
}

static void provisioning_handle_confirmation(uint8_t *packet, uint16_t size){

    UNUSED(size);
    UNUSED(packet);

    // 
    if (prov_emit_output_oob_active){
        prov_emit_output_oob_active = 0;
        provisioning_emit_event(MESH_PB_PROV_STOP_EMIT_OUTPUT_OOB, 1);
    }

    // CalculationInputs
    printf("ConfirmationInputs: ");
    printf_hexdump(prov_confirmation_inputs, sizeof(prov_confirmation_inputs));

    // calculate s1
    btstack_crypto_aes128_cmac_zero(&prov_cmac_request, sizeof(prov_confirmation_inputs), prov_confirmation_inputs, confirmation_salt, &provisioning_handle_confirmation_s1_calculated, NULL);
}

static void provisioning_handle_random_session_nonce_calculated(void * arg){
    UNUSED(arg);

    // The nonce shall be the 13 least significant octets == zero most significant octets
    memset(session_nonce, 0, 3);

    // SessionNonce
    printf("SessionNonce:   ");
    printf_hexdump(session_nonce, sizeof(session_nonce));

    // queue random pdu
    provisioning_queue_pdu(MESH_PROV_RANDOM);

    // next state
    prov_next_command = MESH_PROV_DATA;
}

static void provisioning_handle_random_session_key_calculated(void * arg){
    UNUSED(arg);

    // SessionKey
    printf("SessionKey:   ");
    printf_hexdump(session_key, sizeof(session_key));

    // SessionNonce
    mesh_k1(&prov_cmac_request, dhkey, sizeof(dhkey), provisioning_salt, (const uint8_t*) "prsn", 4, session_nonce, &provisioning_handle_random_session_nonce_calculated, NULL);
}

static void provisioning_handle_random_s1_calculated(void * arg){

    UNUSED(arg);
    
    // ProvisioningSalt
    printf("ProvisioningSalt:   ");
    printf_hexdump(provisioning_salt, sizeof(provisioning_salt));

    // SessionKey
    mesh_k1(&prov_cmac_request, dhkey, sizeof(dhkey), provisioning_salt, (const uint8_t*) "prsk", 4, session_key, &provisioning_handle_random_session_key_calculated, NULL);
}

static void provisioning_handle_random(uint8_t *packet, uint16_t size){

    UNUSED(size);
    UNUSED(packet);

    // TODO: validate Confirmation

    // calc ProvisioningSalt = s1(ConfirmationSalt || RandomProvisioner || RandomDevice)
    memcpy(prov_confirmation_inputs, confirmation_salt, 16);
    memcpy(prov_confirmation_inputs, packet, 16);
    memcpy(prov_confirmation_inputs, random_device, 16);
    btstack_crypto_aes128_cmac_zero(&prov_cmac_request, sizeof(prov_confirmation_inputs), prov_confirmation_inputs, provisioning_salt, &provisioning_handle_random_s1_calculated, NULL);
}

static void provisioning_handle_beacon_key_calculated(void *arg){
    printf("BeaconKey: ");
    printf_hexdump(beacon_key, 16);

    provisioning_timer_stop();

    // queue complete pdu
    provisioning_queue_pdu(MESH_PROV_COMPLETE);
 
    // reset state
    provisioning_done();
}

static void provisioning_handle_s1_for_beacon_key_calculated(void *arg){
    mesh_k1(&prov_cmac_request, net_key, 16, provisioning_salt, id128_tag, sizeof(id128_tag), beacon_key, &provisioning_handle_beacon_key_calculated, NULL);
}

static void provisioning_handle_data_network_id_calculated(void * arg){
    // dump
    printf("Network ID: ");
    printf_hexdump(network_id, 8);

    // calculate s1 for network beacon
    btstack_crypto_aes128_cmac_zero(&prov_cmac_request, 4, (const uint8_t *) "nkbk", provisioning_salt, &provisioning_handle_s1_for_beacon_key_calculated, NULL);
}

static void provisioning_handle_data_device_key(void * arg){
    // calculate Network ID
    mesh_k3(&prov_cmac_request, net_key, network_id, provisioning_handle_data_network_id_calculated, NULL);
}

static void provisioning_handle_data_ccm(void * arg){

    UNUSED(arg);

    // validate MIC?

    // sort provisoning data
    memcpy(net_key, provisioning_data, 16);
    net_key_index = big_endian_read_16(provisioning_data, 16);
    flags = provisioning_data[18];
    iv_index = big_endian_read_32(provisioning_data, 19);
    unicast_address = big_endian_read_16(provisioning_data, 23);

    // dump
    printf("NetKey: ");
    printf_hexdump(provisioning_data, 16);
    printf("NetKeyIndex: %04x\n", net_key_index);
    printf("Flags: %02x\n", flags);
    printf("IVIndex: %04x\n", iv_index);
    printf("UnicastAddress: %02x\n", unicast_address);

    // DeviceKey
    mesh_k1(&prov_cmac_request, dhkey, sizeof(dhkey), provisioning_salt, (const uint8_t*) "prdk", 4, device_key, &provisioning_handle_data_device_key, NULL);
}

static void provisioning_handle_data(uint8_t *packet, uint16_t size){

    UNUSED(size);

    memcpy(enc_provisioning_data, packet, 25);

    // decode response
    btstack_crypo_ccm_init(&prov_ccm_request, session_key, session_nonce, 25);
    btstack_crypto_ccm_decrypt_block(&prov_ccm_request, 25, enc_provisioning_data, provisioning_data, &provisioning_handle_data_ccm, NULL);
}

static void provisioning_handle_pdu(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){

    if (size < 1) return;

    switch (packet_type){
        case HCI_EVENT_PACKET:
            if (packet[0] != HCI_EVENT_MESH_META)  break;
            switch (packet[2]){
                case MESH_PB_ADV_LINK_OPEN:
                    printf("Link opened, reset state\n");
                    provisioning_done();
                    break;
                case MESH_PB_ADV_PDU_SENT:
                    printf("Outgoing packet acked\n");
                    prov_waiting_for_outgoing_complete = 0;
                    provisioning_send_pdu();
                    break;                    
            }
            break;
        case PROVISIONING_DATA_PACKET:
            // check state
            if (prov_next_command == MESH_PROV_INVITE && packet[0] != MESH_PROV_INVITE){
                printf("Ignoring message %u\n", packet[0]);
                break;
            }
            // check expected
            if (packet[0] != prov_next_command){
                printf("Unexpected packet %u, expecting %u\n", packet[0], prov_next_command);
                provisioning_handle_provisioning_error(0x03); // unexpected command
                break;
            }
            // dispatch msg
            switch (packet[0]){
                case MESH_PROV_INVITE:
                    printf("MESH_PROV_INVITE: ");
                    printf_hexdump(packet, size);
                    provisioning_handle_invite(&packet[1], size-1);
                    break;
                case MESH_PROV_START:
                    printf("MESH_PROV_START:  ");
                    printf_hexdump(&packet[1], size-1);
                    provisioning_handle_start(&packet[1], size-1);
                    break;
                case MESH_PROV_PUB_KEY:
                    printf("MESH_PROV_PUB_KEY: ");
                    printf_hexdump(&packet[1], size-1);
                    provisioning_handle_public_key(&packet[1], size-1);
                    break;
                case MESH_PROV_CONFIRM:
                    printf("MESH_PROV_CONFIRM: ");
                    printf_hexdump(&packet[1], size-1);
                    provisioning_handle_confirmation(&packet[1], size-1);
                    break;
                case MESH_PROV_RANDOM:
                    printf("MESH_PROV_RANDOM:  ");
                    printf_hexdump(&packet[1], size-1);
                    provisioning_handle_random(&packet[1], size-1);
                    break;
                case MESH_PROV_DATA:
                    printf("MESH_PROV_DATA:  ");
                    printf_hexdump(&packet[1], size-1);
                    provisioning_handle_data(&packet[1], size-1);
                    break;
                default:
                    printf("TODO: handle provisioning msg type %x\n", packet[0]);
                    printf_hexdump(&packet[1], size-1);
                    break;
            }            
            break;
        default:
            break;
    }
}

static void prov_key_generated(void * arg){
    UNUSED(arg);
    printf("ECC-P256: ");
    dump_data(prov_ec_q, sizeof(prov_ec_q));
    // allow override
    if (prov_public_key_oob_available){
        printf("Replace generated ECC with Public Key OOB:");
        memcpy(prov_ec_q, prov_public_key_oob_q, 64);
        dump_data(prov_ec_q, sizeof(prov_ec_q));
        btstack_crypto_ecc_p256_set_key(prov_public_key_oob_q, prov_public_key_oob_d);
    }
}

void provisioning_device_init(const uint8_t * device_uuid){
    pb_adv_init(device_uuid);
    pb_adv_register_packet_handler(&provisioning_handle_pdu);

    // init provisioning state
    provisioning_done();

    // generate public key
    btstack_crypto_ecc_p256_generate_key(&prov_ecc_p256_request, prov_ec_q, &prov_key_generated, NULL);
}

void provisioning_device_register_packet_handler(btstack_packet_handler_t packet_handler){
    prov_packet_handler = packet_handler;
}

void provisioning_device_set_public_key_oob(const uint8_t * public_key, const uint8_t * private_key){
    prov_public_key_oob_q = public_key;
    prov_public_key_oob_d = private_key;
    prov_public_key_oob_available = 1;
    btstack_crypto_ecc_p256_set_key(prov_public_key_oob_q, prov_public_key_oob_d);
}

void provisioning_device_set_static_oob(uint16_t static_oob_len, const uint8_t * static_oob_data){
    prov_static_oob_available = 1;
    prov_static_oob_data = static_oob_data;
    prov_static_oob_len  = btstack_min(static_oob_len, 16);
}

void provisioning_device_set_output_oob_actions(uint16_t supported_output_oob_action_types, uint8_t max_oob_output_size){
    prov_output_oob_actions = supported_output_oob_action_types;
    prov_output_oob_size    = max_oob_output_size;
}

void provisioning_device_set_input_oob_actions(uint16_t supported_input_oob_action_types, uint8_t max_oob_input_size){
    prov_input_oob_actions = supported_input_oob_action_types;
    prov_input_oob_size    = max_oob_input_size;
}

static void provisioning_device_input_complete(void){
    printf("Input Complete\n");
    provisioning_queue_pdu(MESH_PROV_INPUT_COMPLETE);
    prov_next_command = MESH_PROV_CONFIRM;
}

void provisioning_device_input_oob_complete_numeric(uint16_t pb_adv_cid, uint32_t input_oob){
    UNUSED(pb_adv_cid);
    if (prov_next_command != MESH_PROV_USER_INPUT_OOB) return;

    // store input_oob as auth value
    big_endian_store_32(auth_value, 12, input_oob);

    provisioning_device_input_complete();
}

void provisioning_device_input_oob_complete_alphanumeric(uint16_t pb_adv_cid, const uint8_t * input_oob_data, uint16_t input_oob_len){
    UNUSED(pb_adv_cid);
    printf("provisioning_device_input_oob_complete_alphanumeric, next %u, expect %u\n", prov_next_command, MESH_PROV_USER_INPUT_OOB);
    if (prov_next_command != MESH_PROV_USER_INPUT_OOB) return;

    // store input_oob and fillup with zeros
    input_oob_len = btstack_min(input_oob_len, 16);
    memset(auth_value, 0, 16);
    memcpy(auth_value, input_oob_data, input_oob_len);

    provisioning_device_input_complete();
}


uint8_t provisioning_device_data_get_flags(void){
    return flags;
}
const uint8_t * provisioning_device_data_get_network_id(void){
    return network_id;
}
uint32_t provisioning_device_data_get_iv_index(void){
    return iv_index;
}
const uint8_t * provisioning_device_data_get_beacon_key(void){
    return beacon_key;
}
