#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CppUTest/TestHarness.h"
#include "CppUTest/CommandLineTestRunner.h"

#include "btstack_run_loop_posix.h"

#include "hci_cmd.h"
#include "btstack_util.h"
#include "btstack_debug.h"
#include "btstack_memory.h"
#include "btstack_crypto.h"
#include "hci.h"
#include "hci_dump.h"

void mock_init(void);
void mock_simulate_hci_event(uint8_t * packet, uint16_t size);
void aes128_report_result(void);
uint8_t * mock_packet_buffer(void);
uint16_t mock_packet_buffer_len(void);
void mock_clear_packet_buffer(void);

void CHECK_EQUAL_ARRAY(uint8_t * expected, uint8_t * actual, int size){
	int i;
	for (i=0; i<size; i++){
		if (expected[i] != actual[i]) {
			printf("offset %u wrong\n", i);
			printf("expected: "); printf_hexdump(expected, size);
			printf("actual:   "); printf_hexdump(actual, size);
		}
		BYTES_EQUAL(expected[i], actual[i]);
	}
}

static int parse_hex(uint8_t * buffer, const char * hex_string){
    int len = 0;
    while (*hex_string){
        if (*hex_string == ' '){
            hex_string++;
            continue;
        }
        int high_nibble = nibble_for_char(*hex_string++);
        int low_nibble = nibble_for_char(*hex_string++);
        *buffer++ = (high_nibble << 4) | low_nibble;
        len++;
    }
    return len;
}

#if 0
static void validate_message(const char * name, const char * message_string, const char * cmac_string){

    btstack_crypto_aes128_cmac_t btstack_crypto_aes128_cmac;
    
    mock_clear_packet_buffer();
    int len = parse_hex(m, message_string);

    // expected result
    sm_key_t cmac;
    parse_hex(cmac, cmac_string);

    printf("-- verify key %s message %s, len %u:\nm:    %s\ncmac: %s\n", key_string, name, len, message_string, cmac_string);

    sm_key_t key;
    parse_hex(key, key_string);
    // printf_hexdump(key, 16);

    cmac_hash_received = 0;
    btstack_crypto_aes128_cmac_message(&btstack_crypto_aes128_cmac, key, len, m, cmac_hash, &cmac_done2, NULL);
    while (!cmac_hash_received){
        aes128_report_result();
    }
    CHECK_EQUAL_ARRAY(cmac, cmac_hash, 16);
}

#define VALIDATE_MESSAGE(NAME) validate_message(#NAME, NAME##_string, cmac_##NAME##_string)
#endif

static uint8_t zero[16] = { 0 };
static btstack_crypto_aes128_t crypto_aes128_request;
static uint8_t ciphertext[16];

static int crypto_done;
static void crypto_done_callback(void * arg){
    printf_hexdump(ciphertext, 16);
    crypto_done = 1;
}
static void perform_crypto_operation(void){
    crypto_done = 0;
    while (!crypto_done){
        aes128_report_result();
    }
}

TEST_GROUP(Crypto){
	void setup(void){
        static int first = 1;
        if (first){
            first = 0;
            btstack_memory_init();
            btstack_run_loop_init(btstack_run_loop_posix_get_instance());            
        }
        btstack_crypto_init();
    }
};

TEST(Crypto, AES128){
    mock_init();
    btstack_crypto_aes128_encrypt(&crypto_aes128_request, zero, zero, ciphertext, crypto_done_callback, NULL);
    perform_crypto_operation();
    uint8_t expected[16];
    parse_hex(expected, "66e94bd4ef8a2c3b884cfa59ca342b2e");
    CHECK_EQUAL_ARRAY(expected, ciphertext, 16);
}

int main (int argc, const char * argv[]){
    // hci_dump_open("hci_dump.pklg", HCI_DUMP_PACKETLOGGER);
    return CommandLineTestRunner::RunAllTests(argc, argv);
}
