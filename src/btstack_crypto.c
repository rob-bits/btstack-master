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

// configure ECC implementations
#ifdef ENABLE_LE_SECURE_CONNECTIONS
#if defined(ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS) && defined(HAVE_MBEDTLS_ECC_P256)
#error "If you already have mbedTLS (HAVE_MBEDTLS_ECC_P256), please disable uECC (ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS) in bstack_config.h"
#endif
#ifdef ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS
#define USE_SOFTWARE_ECDH_IMPLEMENTATION
#define USE_MICRO_ECC_FOR_ECDH
#endif
#ifdef HAVE_MBEDTLS_ECC_P256
#define USE_SOFTWARE_ECDH_IMPLEMENTATION
#define USE_MBEDTLS_FOR_ECDH
#endif
#endif /* ENABLE_LE_SECURE_CONNECTIONS */

// Software ECDH implementation provided by micro-ecc
#ifdef USE_MICRO_ECC_FOR_ECDH
#include "uECC.h"
#endif

// Software ECDH implementation provided by mbedTLS
#ifdef USE_MBEDTLS_FOR_ECDH
#include "mbedtls/config.h"
#include "mbedtls/platform.h"
#include "mbedtls/ecp.h"
#endif

// Software ECDH implementation provided by mbedtls
#ifdef USE_MBEDTLS_FOR_ECDH
static mbedtls_ecp_group   mbedtls_ec_group;
#endif

typedef enum {
    CMAC_IDLE,
    CMAC_CALC_SUBKEYS,
    CMAC_W4_SUBKEYS,
    CMAC_CALC_MI,
    CMAC_W4_MI,
    CMAC_CALC_MLAST,
    CMAC_W4_MLAST
} btstack_crypto_cmac_state_t;

typedef enum {
    EC_KEY_GENERATION_IDLE,
    EC_KEY_GENERATION_GENERATING_RANDOM,
    EC_KEY_GENERATION_ACTIVE,
    EC_KEY_GENERATION_W4_KEY,
    EC_KEY_GENERATION_DONE,
} btstack_crypto_ec_key_generation_state_t;

static void btstack_crypto_run(void);

static uint8_t btstack_crypto_initialized;
static btstack_linked_list_t btstack_crypto_operations;
static btstack_packet_callback_registration_t hci_event_callback_registration;
static uint8_t btstack_crypto_wait_for_hci_result;

// state for AES-CMAC
static btstack_crypto_cmac_state_t sm_cmac_state;
static sm_key_t     sm_cmac_k;
static sm_key_t     sm_cmac_x;
static sm_key_t     sm_cmac_m_last;
static uint8_t      sm_cmac_block_current;
static uint8_t      sm_cmac_block_count;

#ifdef ENABLE_LE_SECURE_CONNECTIONS
// state for ECC-P192
static uint8_t                                  btstack_crypto_ec_p192_public_key[64];
static uint8_t                                  btstack_crypto_ec_p192_random[64];
static uint8_t                                  btstack_crypto_ec_p192_random_len;
static uint8_t                                  btstack_crypto_ec_p192_random_offset;
static btstack_crypto_ec_key_generation_state_t btstack_crypto_ec_p192_key_generation_state;

#ifdef USE_SOFTWARE_ECDH_IMPLEMENTATION
static uint8_t ec_d[32];
#endif
#endif

static inline void btstack_crypto_cmac_next_state(void){
    sm_cmac_state = (btstack_crypto_cmac_state_t) (((int)sm_cmac_state) + 1);
}

static int btstack_crytpo_cmac_last_block_complete(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac){
	uint16_t len = btstack_crypto_cmac->size;
    if (len == 0) return 0;
    return (len & 0x0f) == 0;
}

static void btstack_crypto_aes128_start(const sm_key_t key, const sm_key_t plaintext){
 	uint8_t key_flipped[16];
 	uint8_t plaintext_flipped[16];
    reverse_128(key, key_flipped);
    reverse_128(plaintext, plaintext_flipped);
 	btstack_crypto_wait_for_hci_result = 1;
    hci_send_cmd(&hci_le_encrypt, key_flipped, plaintext_flipped);
}

static uint8_t btstack_crypto_cmac_get_byte(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac, uint16_t pos){
	if (btstack_crypto_cmac->btstack_crypto.operation == BTSTACK_CRYPTO_CMAC_GENERATOR){
		return (*btstack_crypto_cmac->get_byte_callback)(pos);
	} else {
		return btstack_crypto_cmac->message[pos]; 
	}
}

static void btstack_crytpo_cmac_handle_aes_engine_ready(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac){
    switch (sm_cmac_state){
        case CMAC_CALC_SUBKEYS: {
            sm_key_t const_zero;
            memset(const_zero, 0, 16);
            btstack_crypto_cmac_next_state();
            btstack_crypto_aes128_start(sm_cmac_k, const_zero);
            break;
        }
        case CMAC_CALC_MI: {
            int j;
            sm_key_t y;
            for (j=0;j<16;j++){
                y[j] = sm_cmac_x[j] ^ btstack_crypto_cmac_get_byte(btstack_crypto_cmac, sm_cmac_block_current*16 + j);
            }
            sm_cmac_block_current++;
            btstack_crypto_cmac_next_state();
            btstack_crypto_aes128_start(sm_cmac_k, y);
            break;
        }
        case CMAC_CALC_MLAST: {
            int i;
            sm_key_t y;
            for (i=0;i<16;i++){
                y[i] = sm_cmac_x[i] ^ sm_cmac_m_last[i];
            }
            sm_cmac_block_current++;
            btstack_crypto_cmac_next_state();
            btstack_crypto_aes128_start(sm_cmac_k, y);
            break;
        }
        default:
            log_info("btstack_crytpo_cmac_handle_aes_engine_ready called in state %u", sm_cmac_state);
            break;
    }
}

static void btstack_crypto_cmac_shift_left_by_one_bit_inplace(int len, uint8_t * data){
    int i;
    int carry = 0;
    for (i=len-1; i >= 0 ; i--){
        int new_carry = data[i] >> 7;
        data[i] = data[i] << 1 | carry;
        carry = new_carry;
    }
}

static void btstack_crypto_cmac_handle_encryption_result(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac, sm_key_t data){
    switch (sm_cmac_state){
        case CMAC_W4_SUBKEYS: {
            sm_key_t k1;
            memcpy(k1, data, 16);
            btstack_crypto_cmac_shift_left_by_one_bit_inplace(16, k1);
            if (data[0] & 0x80){
                k1[15] ^= 0x87;
            }
            sm_key_t k2;
            memcpy(k2, k1, 16);
            btstack_crypto_cmac_shift_left_by_one_bit_inplace(16, k2);
            if (k1[0] & 0x80){
                k2[15] ^= 0x87;
            }

            log_info_key("k", sm_cmac_k);
            log_info_key("k1", k1);
            log_info_key("k2", k2);

            // step 4: set m_last
            int i;
            if (btstack_crytpo_cmac_last_block_complete(btstack_crypto_cmac)){
                for (i=0;i<16;i++){
                    sm_cmac_m_last[i] = btstack_crypto_cmac_get_byte(btstack_crypto_cmac, btstack_crypto_cmac->size - 16 + i) ^ k1[i];
                }
            } else {
                int valid_octets_in_last_block = btstack_crypto_cmac->size & 0x0f;
                for (i=0;i<16;i++){
                    if (i < valid_octets_in_last_block){
                        sm_cmac_m_last[i] = btstack_crypto_cmac_get_byte(btstack_crypto_cmac, (btstack_crypto_cmac->size & 0xfff0) + i) ^ k2[i];
                        continue;
                    }
                    if (i == valid_octets_in_last_block){
                        sm_cmac_m_last[i] = 0x80 ^ k2[i];
                        continue;
                    }
                    sm_cmac_m_last[i] = k2[i];
                }
            }

            // next
            sm_cmac_state = sm_cmac_block_current < sm_cmac_block_count - 1 ? CMAC_CALC_MI : CMAC_CALC_MLAST;
            break;
        }
        case CMAC_W4_MI:
            memcpy(sm_cmac_x, data, 16);
            sm_cmac_state = sm_cmac_block_current < sm_cmac_block_count - 1 ? CMAC_CALC_MI : CMAC_CALC_MLAST;
            break;
        case CMAC_W4_MLAST:
            // done
            log_info("Setting CMAC Engine to IDLE");
            sm_cmac_state = CMAC_IDLE;
            log_info_key("CMAC", data);
            memcpy(btstack_crypto_cmac->hash, data, 16);
			btstack_linked_list_pop(&btstack_crypto_operations);
			(*btstack_crypto_cmac->btstack_crypto.context_callback.callback)(btstack_crypto_cmac->btstack_crypto.context_callback.context);
            break;
        default:
            log_info("btstack_crypto_cmac_handle_encryption_result called in state %u", sm_cmac_state);
            break;
    }
}

static void btstack_crypo_cmac_start(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac){

    memcpy(sm_cmac_k, btstack_crypto_cmac->key, 16);
    memset(sm_cmac_x, 0, 16);
    sm_cmac_block_current = 0;

    // step 2: n := ceil(len/const_Bsize);
    sm_cmac_block_count = (btstack_crypto_cmac->size + 15) / 16;

    // step 3: ..
    if (sm_cmac_block_count==0){
        sm_cmac_block_count = 1;
    }
    log_info("btstack_crypo_cmac_start: len %u, block count %u", btstack_crypto_cmac->size, sm_cmac_block_count);

    // first, we need to compute l for k1, k2, and m_last
    sm_cmac_state = CMAC_CALC_SUBKEYS;

    // let's go
    btstack_crytpo_cmac_handle_aes_engine_ready(btstack_crypto_cmac);
}

#ifdef ENABLE_LE_SECURE_CONNECTIONS

#if (defined(USE_MICRO_ECC_FOR_ECDH) && !defined(WICED_VERSION)) || defined(USE_MBEDTLS_FOR_ECDH)
// @return OK
static int sm_generate_f_rng(unsigned char * buffer, unsigned size){
    if (btstack_crypto_ec_p192_key_generation_state != EC_KEY_GENERATION_ACTIVE) return 0;
    log_info("sm_generate_f_rng: size %u - offset %u", (int) size, btstack_crypto_ec_p192_random_offset);
    while (size) {
        *buffer++ = btstack_crypto_ec_p192_random[btstack_crypto_ec_p192_random_offset++];
        size--;
    }
    return 1;
}
#endif
#ifdef USE_MBEDTLS_FOR_ECDH
// @return error - just wrap sm_generate_f_rng
static int sm_generate_f_rng_mbedtls(void * context, unsigned char * buffer, size_t size){
    UNUSED(context);
    return sm_generate_f_rng(buffer, size) == 0;
}
#endif /* USE_MBEDTLS_FOR_ECDH */

static void btstack_crypto_ec_p192_generate_key_software(void){

    btstack_crypto_ec_p192_random_offset = 0;
    
    // generate EC key
#ifdef USE_MICRO_ECC_FOR_ECDH

#ifndef WICED_VERSION
    log_info("set uECC RNG for initial key generation with 64 random bytes");
    // micro-ecc from WICED SDK uses its wiced_crypto_get_random by default - no need to set it
    uECC_set_rng(&sm_generate_f_rng);
#endif /* WICED_VERSION */

#if uECC_SUPPORTS_secp256r1
    // standard version
    uECC_make_key(btstack_crypto_ec_p192_public_key, ec_d, uECC_secp256r1());

    // disable RNG again, as returning no randmon data lets shared key generation fail
    log_info("disable uECC RNG in standard version after key generation");
    uECC_set_rng(NULL);
#else
    // static version
    uECC_make_key(btstack_crypto_ec_p192_public_key, ec_d);
#endif
#endif /* USE_MICRO_ECC_FOR_ECDH */

#ifdef USE_MBEDTLS_FOR_ECDH
    mbedtls_mpi d;
    mbedtls_ecp_point P;
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&P);
    int res = mbedtls_ecp_gen_keypair(&mbedtls_ec_group, &d, &P, &sm_generate_f_rng_mbedtls, NULL);
    log_info("gen keypair %x", res);
    mbedtls_mpi_write_binary(&P.X, &btstack_crypto_ec_p192_public_key[0],  32);
    mbedtls_mpi_write_binary(&P.Y, &btstack_crypto_ec_p192_public_key[32], 32);
    mbedtls_mpi_write_binary(&d, ec_d, 32);
    mbedtls_ecp_point_free(&P);
    mbedtls_mpi_free(&d);
#endif  /* USE_MBEDTLS_FOR_ECDH */
}

static void btstack_crypto_ec_p192_calculate_dhkey_software(btstack_crypto_ec_p192_t * btstack_crypto_ec_p192){
    memset(btstack_crypto_ec_p192->dhkey, 0, 32);

#ifdef USE_MICRO_ECC_FOR_ECDH
#if uECC_SUPPORTS_secp256r1
    // standard version
    uECC_shared_secret(btstack_crypto_ec_p192->public_key, ec_d, btstack_crypto_ec_p192->dhkey, uECC_secp256r1());
#else
    // static version
    uECC_shared_secret(btstack_crypto_ec_p192->public_key, ec_d, btstack_crypto_ec_p192->dhkey);
#endif
#endif

#ifdef USE_MBEDTLS_FOR_ECDH
    // da * Pb
    mbedtls_mpi d;
    mbedtls_ecp_point Q;
    mbedtls_ecp_point DH;
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);
    mbedtls_ecp_point_init(&DH);
    mbedtls_mpi_read_binary(&d, ec_d, 32);
    mbedtls_mpi_read_binary(&Q.X, &btstack_crypto_ec_p192->public_key[0] , 32);
    mbedtls_mpi_read_binary(&Q.Y, &btstack_crypto_ec_p192->public_key[32], 32);
    mbedtls_mpi_lset(&Q.Z, 1);
    mbedtls_ecp_mul(&mbedtls_ec_group, &DH, &d, &Q, NULL, NULL);
    mbedtls_mpi_write_binary(&DH.X, btstack_crypto_ec_p192->dhkey, 32);
    mbedtls_ecp_point_free(&DH);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);
#endif

    log_info("dhkey");
    log_info_hexdump(btstack_crypto_ec_p192->dhkey, 32);
}

#endif


static void btstack_crypto_run(void){

    btstack_crypto_aes128_t        * btstack_crypto_aes128;
    btstack_crypto_aes128_cmac_t   * btstack_crypto_cmac;
    btstack_crypto_ec_p192_t       * btstack_crypto_ec_p192;

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
			btstack_crypto_aes128 = (btstack_crypto_aes128_t *) btstack_crypto;
			btstack_crypto_aes128_start(btstack_crypto_aes128->key, btstack_crypto_aes128->plaintext);
		    break;
		case BTSTACK_CRYPTO_CMAC_MESSAGE:
		case BTSTACK_CRYPTO_CMAC_GENERATOR:
			btstack_crypto_wait_for_hci_result = 1;
			btstack_crypto_cmac = (btstack_crypto_aes128_cmac_t *) btstack_crypto;
			if (sm_cmac_state == CMAC_IDLE){
				btstack_crypo_cmac_start(btstack_crypto_cmac);
			} else {
				btstack_crytpo_cmac_handle_aes_engine_ready(btstack_crypto_cmac);
			}
			break;
#ifdef ENABLE_LE_SECURE_CONNECTIONS
        case BTSTACK_CRYPTO_EC_P192_GENERATE_KEY:
            btstack_crypto_ec_p192 = (btstack_crypto_ec_p192_t *) btstack_crypto;
            switch (btstack_crypto_ec_p192_key_generation_state){
                case EC_KEY_GENERATION_DONE:
                    // done
                    memcpy(btstack_crypto_ec_p192->public_key, btstack_crypto_ec_p192_public_key, 64);
                    btstack_linked_list_pop(&btstack_crypto_operations);
                    (*btstack_crypto_ec_p192->btstack_crypto.context_callback.callback)(btstack_crypto_ec_p192->btstack_crypto.context_callback.context);                    
                    break;
                case EC_KEY_GENERATION_IDLE:
                    log_info("start ecc random");
                    btstack_crypto_ec_p192_key_generation_state = EC_KEY_GENERATION_GENERATING_RANDOM;
                    btstack_crypto_ec_p192_random_offset = 0;
                    btstack_crypto_wait_for_hci_result = 1;
                    hci_send_cmd(&hci_le_rand);
                    break;
                case EC_KEY_GENERATION_GENERATING_RANDOM:
                    log_info("more ecc random");
                    btstack_crypto_wait_for_hci_result = 1;
                    hci_send_cmd(&hci_le_rand);
                    break;
                default:
                    break;
            }
            break;
        case BTSTACK_CRYPTO_EC_P192_CALCULATE_DHKEY:
            btstack_crypto_ec_p192 = (btstack_crypto_ec_p192_t *) btstack_crypto;
            btstack_crypto_ec_p192_calculate_dhkey_software(btstack_crypto_ec_p192);
            // done
            btstack_linked_list_pop(&btstack_crypto_operations);
            (*btstack_crypto_ec_p192->btstack_crypto.context_callback.callback)(btstack_crypto_ec_p192->btstack_crypto.context_callback.context);                    
            break;        
#endif
		default:
			break;
	}
}

static void btstack_crypto_handle_random_data(const uint8_t * data, uint16_t len){
    btstack_crypto_random_t * btstack_crypto_random;

    btstack_crypto_wait_for_hci_result = 0;
    btstack_crypto_t        * btstack_crypto = (btstack_crypto_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
	if (!btstack_crypto) return;
    switch (btstack_crypto->operation){
        case BTSTACK_CRYPTO_RANDOM:
            btstack_crypto_random = (btstack_crypto_random_t*) btstack_crypto;
            uint16_t bytes_to_copy = btstack_min(btstack_crypto_random->size, len);
            memcpy(btstack_crypto_random->buffer, data, bytes_to_copy);
            btstack_crypto_random->buffer += bytes_to_copy;
            btstack_crypto_random->size   -= bytes_to_copy;
            // data processed, more?
            if (!btstack_crypto_random->size) {
                // done
                btstack_linked_list_pop(&btstack_crypto_operations);
                (*btstack_crypto_random->btstack_crypto.context_callback.callback)(btstack_crypto_random->btstack_crypto.context_callback.context);
            }
            break;
#ifdef ENABLE_LE_SECURE_CONNECTIONS
        case BTSTACK_CRYPTO_EC_P192_GENERATE_KEY:
            memcpy(&btstack_crypto_ec_p192_random[btstack_crypto_ec_p192_random_len], data, 8);
            btstack_crypto_ec_p192_random_len += 8;
            if (btstack_crypto_ec_p192_random_len >= 64) {
                btstack_crypto_ec_p192_key_generation_state = EC_KEY_GENERATION_ACTIVE;
                btstack_crypto_ec_p192_generate_key_software();
                log_info("Elliptic curve: X");
                log_info_hexdump(&btstack_crypto_ec_p192_public_key[0],32);
                log_info("Elliptic curve: Y");
                log_info_hexdump(&btstack_crypto_ec_p192_public_key[32],32);
                btstack_crypto_ec_p192_key_generation_state = EC_KEY_GENERATION_DONE;
            }
            break;
#endif
        default:
            break;
    }
	// more work?
	btstack_crypto_run();
}

static void btstack_crypto_handle_encryption_result(const uint8_t * data){
	btstack_crypto_wait_for_hci_result = 0;
	btstack_crypto_aes128_t * btstack_crypto_aes128;
	btstack_crypto_aes128_cmac_t * btstack_crypto_cmac;
	uint8_t result[16];
    btstack_crypto_t * btstack_crypto = (btstack_crypto_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
	if (!btstack_crypto) return;
	switch (btstack_crypto->operation){
		case BTSTACK_CRYPTO_AES128:
			btstack_crypto_aes128 = (btstack_crypto_aes128_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
		    reverse_128(data, btstack_crypto_aes128->ciphertext);
			// done
			btstack_linked_list_pop(&btstack_crypto_operations);
			(*btstack_crypto_aes128->btstack_crypto.context_callback.callback)(btstack_crypto_aes128->btstack_crypto.context_callback.context);
			break;
		case BTSTACK_CRYPTO_CMAC_GENERATOR:
		case BTSTACK_CRYPTO_CMAC_MESSAGE:
			btstack_crypto_cmac = (btstack_crypto_aes128_cmac_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
		    reverse_128(data, result);
		    btstack_crypto_cmac_handle_encryption_result(btstack_crypto_cmac, result);
			break;
		default:
			break;
	}
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
	    }
	    if (HCI_EVENT_IS_COMMAND_COMPLETE(packet, hci_le_rand)){
	        btstack_crypto_handle_random_data(&packet[6], 8);
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

void btstack_crypto_aes128_cmac_generator(btstack_crypto_aes128_cmac_t * request, const uint8_t * key, uint16_t size, uint8_t (*get_byte_callback)(uint16_t pos), uint8_t * hash, void (* callback)(void * arg), void * callback_arg){
	request->btstack_crypto.context_callback.callback  = callback;
	request->btstack_crypto.context_callback.context   = callback_arg;
	request->btstack_crypto.operation         		   = BTSTACK_CRYPTO_CMAC_GENERATOR;
	request->key 									   = key;
	request->size 									   = size;
	request->get_byte_callback 						   = get_byte_callback;
	request->hash 									   = hash;
	btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
	btstack_crypto_run();
}

void btstack_crypto_aes128_cmac_message(btstack_crypto_aes128_cmac_t * request, const uint8_t * key, uint16_t size, const uint8_t * message, uint8_t * hash, void (* callback)(void * arg), void * callback_arg){
	request->btstack_crypto.context_callback.callback  = callback;
	request->btstack_crypto.context_callback.context   = callback_arg;
	request->btstack_crypto.operation         		   = BTSTACK_CRYPTO_CMAC_MESSAGE;
	request->key 									   = key;
	request->size 									   = size;
	request->message        						   = message;
	request->hash 									   = hash;
	btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
	btstack_crypto_run();
}

#ifdef ENABLE_LE_SECURE_CONNECTIONS
void btstack_crypto_ec_p192_generate_key(btstack_crypto_ec_p192_t * request, uint8_t * public_key, void (* callback)(void * arg), void * callback_arg){
    request->btstack_crypto.context_callback.callback  = callback;
    request->btstack_crypto.context_callback.context   = callback_arg;
    request->btstack_crypto.operation                  = BTSTACK_CRYPTO_EC_P192_GENERATE_KEY;
    request->public_key                                = public_key;
    btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
    btstack_crypto_run();
}

void btstack_crypto_ec_p192_calculate_dhkey(btstack_crypto_ec_p192_t * request, const uint8_t * public_key, uint8_t * dhkey, void (* callback)(void * arg), void * callback_arg){
    request->btstack_crypto.context_callback.callback  = callback;
    request->btstack_crypto.context_callback.context   = callback_arg;
    request->btstack_crypto.operation                  = BTSTACK_CRYPTO_EC_P192_CALCULATE_DHKEY;
    request->public_key                                = (uint8_t *) public_key;
    request->dhkey                                     = dhkey;
    btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
    btstack_crypto_run();
}
int btstack_crypto_ec_p192_validate_public_key(const uint8_t * public_key){

    // validate public key using micro-ecc
    int err = 0;

#ifdef USE_MICRO_ECC_FOR_ECDH
#if uECC_SUPPORTS_secp256r1
    // standard version
    err = uECC_valid_public_key(public_key, uECC_secp256r1()) == 0;
#else
    // static version
    err = uECC_valid_public_key(public_key) == 0;
#endif
#endif

#ifdef USE_MBEDTLS_FOR_ECDH
    mbedtls_ecp_point Q;
    mbedtls_ecp_point_init( &Q );
    mbedtls_mpi_read_binary(&Q.X, &public_key[0], 32);
    mbedtls_mpi_read_binary(&Q.Y, &public_key[32], 32);
    mbedtls_mpi_lset(&Q.Z, 1);
    err = mbedtls_ecp_check_pubkey(&mbedtls_ec_group, &Q);
    mbedtls_ecp_point_free( & Q);
#endif

    if (err){
        log_error("public key invalid %x", err);
    }
    return  err;
}
#endif
