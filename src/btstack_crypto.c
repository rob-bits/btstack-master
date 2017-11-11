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
 *
 * THIS SOFTWARE IS PROVIDED BY MATTHIAS RINGWALD AND CONTRIBUTORS
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
 */

#define __BTSTACK_FILE__ "btstack_crypto.c"

/*
 * btstack_crypto.h
 *
 * Central place for all crypto-related functions with completion callbacks to allow
 * using of MCU crypto peripherals or the Bluetooth controller
 */

#include "btstack_crypto.h"

#include "btstack_debug.h"
#include "btstack_event.h"
#include "btstack_linked_list.h"
#include "btstack_util.h"
#include "hci.h"

static uint8_t btstack_crypto_initialized;
static btstack_linked_list_t btstack_crypto_operations;
static btstack_packet_callback_registration_t hci_event_callback_registration;
static uint8_t btstack_crypto_wait_for_hci_result;

static void btstack_crypto_run(void){

    uint8_t key_flipped[16];
    uint8_t plaintext_flipped[16];
    btstack_crypto_aes128_t * btstack_crypto_aes128;

	// already active?
	if (btstack_crypto_wait_for_hci_result) return;

	// anything to do?
	if (btstack_linked_list_empty(&btstack_crypto_operations)) return;

    // can send a command?
    if (!hci_can_send_command_packet_now()) return;

	btstack_crypto_t * btstack_crypto = (btstack_crypto_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
	switch (btstack_crypto->operation){
		case BTSTACK_CRYPTO_RANDOM:
			btstack_crypto_wait_for_hci_result = 1;
		    hci_send_cmd(&hci_le_rand);
		    break;
		case BTSTACK_CRYPTO_AES128:
			btstack_crypto_wait_for_hci_result = 1;
			btstack_crypto_aes128 = (btstack_crypto_aes128_t *) btstack_crypto;
		    reverse_128(btstack_crypto_aes128->key, key_flipped);
		    reverse_128(btstack_crypto_aes128->plaintext, plaintext_flipped);
		    hci_send_cmd(&hci_le_encrypt, key_flipped, plaintext_flipped);
		    break;
		default:
			break;
	}
}

static void btstack_crypto_handle_random_data(const uint8_t * data, uint16_t len){
	btstack_crypto_wait_for_hci_result = 0;
	btstack_crypto_random_t * btstack_crypto_random = (btstack_crypto_random_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
	if (btstack_crypto_random->btstack_crypto.operation != BTSTACK_CRYPTO_RANDOM) return;
	uint16_t bytes_to_copy = btstack_min(btstack_crypto_random->size, len);
	memcpy(btstack_crypto_random->buffer, data, bytes_to_copy);
	btstack_crypto_random->buffer += len;
	btstack_crypto_random->size   -= len;
	// data processed, more?
	if (!btstack_crypto_random->size) {
		// done
		btstack_linked_list_pop(&btstack_crypto_operations);
		(*btstack_crypto_random->btstack_crypto.context_callback.callback)(btstack_crypto_random->btstack_crypto.context_callback.context);
	}
	// more work?
	btstack_crypto_run();
}

static void btstack_crypto_handle_encryption_result(const uint8_t * data){
	btstack_crypto_wait_for_hci_result = 0;
	btstack_crypto_aes128_t * btstack_crypto_aes128 = (btstack_crypto_aes128_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
	if (btstack_crypto_aes128->btstack_crypto.operation != BTSTACK_CRYPTO_AES128) return;
    reverse_128(data, btstack_crypto_aes128->ciphertext);
	// done
	btstack_linked_list_pop(&btstack_crypto_operations);
	(*btstack_crypto_aes128->btstack_crypto.context_callback.callback)(btstack_crypto_aes128->btstack_crypto.context_callback.context);
	// more work?
	btstack_crypto_run();
}

static void btstack_crypto_event_handler(uint8_t packet_type, uint16_t cid, uint8_t *packet, uint16_t size){
    UNUSED(packet_type); // ok: registered with hci_event_callback_registration
    UNUSED(cid);         // ok: there is no channel
    UNUSED(size);        // ok: fixed format events read from HCI buffer

    if (packet_type   			          != HCI_EVENT_PACKET) return;
    if (hci_get_state() 			      != HCI_STATE_WORKING) return;

    if (hci_event_packet_get_type(packet) == HCI_EVENT_COMMAND_COMPLETE){
	    if (HCI_EVENT_IS_COMMAND_COMPLETE(packet, hci_le_encrypt)){
	        btstack_crypto_handle_encryption_result(&packet[6]);
	        return;
	    }
	    if (HCI_EVENT_IS_COMMAND_COMPLETE(packet, hci_le_rand)){
	        btstack_crypto_handle_random_data(&packet[6], 8);
	        return;
	    }
    }

    // try processing
	btstack_crypto_run();    
}

void btstack_crypto_init(void){
	if (btstack_crypto_initialized) return;
	btstack_crypto_initialized = 1;
	// register with HCI
    hci_event_callback_registration.callback = &btstack_crypto_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);
}

void btstack_crypto_random_generate(btstack_crypto_random_t * request, uint8_t * buffer, uint16_t size, void (* callback)(void * arg), void * callback_arg){
	request->btstack_crypto.context_callback.callback  = callback;
	request->btstack_crypto.context_callback.context   = callback_arg;
	request->btstack_crypto.operation         		   = BTSTACK_CRYPTO_RANDOM;
	request->buffer = buffer;
	request->size   = size;
	btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
	btstack_crypto_run();
}

void btstack_crypto_aes128_encrypt(btstack_crypto_aes128_t * request, const uint8_t * key, const uint8_t * plaintext, uint8_t * ciphertext, void (* callback)(void * arg), void * callback_arg){
	request->btstack_crypto.context_callback.callback  = callback;
	request->btstack_crypto.context_callback.context   = callback_arg;
	request->btstack_crypto.operation         		   = BTSTACK_CRYPTO_AES128;
	request->key 									   = key;
	request->plaintext      					       = plaintext;
	request->ciphertext 							   = ciphertext;
	btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
	btstack_crypto_run();
}
