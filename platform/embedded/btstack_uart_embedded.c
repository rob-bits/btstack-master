/*
 * Copyright (C) 2016 BlueKitchen GmbH
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

#define __BTSTACK_FILE__ "btstack_uart_embedded.c"

/*
 *  btstack_uart_embedded.c
 *
 *  Common code to access UART via asynchronous block read/write commands on top of hal_uart_dma.h
 *
 */

#include "btstack_debug.h"
#include "btstack_uart.h"
#include "btstack_run_loop_embedded.h"
#include "hal_uart_dma.h"

#ifdef ENABLE_H5
#include "btstack_slip.h"
// max size of outgoing SLIP chunks 
#define SLIP_TX_CHUNK_LEN   128
static void btstack_uart_embedded_encode_chunk_and_send(void);
#endif

// uart config
static const btstack_uart_config_t * uart_config;

// data source for integration with BTstack Runloop
static btstack_data_source_t transport_data_source;

static int send_complete;
static int receive_complete;
static int wakeup_event;

// callbacks
static void (*block_sent)(void);
static void (*block_received)(void);
static void (*wakeup_handler)(void);

#ifdef ENABLE_H5

static void (*frame_sent)(void);
static void (*frame_received)(uint16_t frame_size);

// encoded SLIP chunk
static uint8_t  btstack_uart_embedded_slip_outgoing_buffer[SLIP_TX_CHUNK_LEN];
// != 0 if 
static uint16_t received_frame_size;
static int      slip_outgoing_active;
#endif

static void btstack_uart_embedded_block_received(void){
    receive_complete = 1;
    btstack_run_loop_embedded_trigger();
}

#ifdef ENABLE_H5
static void btstack_uart_embedded_frame_received(uint16_t frame_size){
    received_frame_size = frame_size;
    btstack_run_loop_embedded_trigger();
}
#endif

static void btstack_uart_embedded_block_sent(void){
    send_complete = 1;
    btstack_run_loop_embedded_trigger();
}

static void btstack_uart_embedded_cts_pulse(void){
    wakeup_event = 1;
    btstack_run_loop_embedded_trigger();
}

static int btstack_uart_embedded_init(const btstack_uart_config_t * config){
    uart_config = config;
    hal_uart_dma_set_block_received(&btstack_uart_embedded_block_received);
    hal_uart_dma_set_block_sent(&btstack_uart_embedded_block_sent);
#ifdef ENABLE_H5
    hal_uart_dma_set_frame_received(&btstack_uart_embedded_frame_received);
#endif    
    return 0;
}

static void btstack_uart_embedded_process(btstack_data_source_t *ds, btstack_data_source_callback_type_t callback_type) {
    switch (callback_type){
        case DATA_SOURCE_CALLBACK_POLL:
            if (send_complete){
                send_complete = 0;
#ifdef ENABLE_H5
                if (slip_outgoing_active){
                    if (btstack_slip_encoder_has_data()){
                        btstack_uart_embedded_encode_chunk_and_send();
                        return;
                    }
                    slip_outgoing_active = 0;
                    if (frame_sent){
                        frame_sent();
                    }
                    return;         
                }
#endif
                if (block_sent){
                    block_sent();
                }
            }
            if (receive_complete){
                receive_complete = 0;
                if (block_received){
                    block_received();
                }
            }
#ifdef ENABLE_H5
            if (received_frame_size){
                uint16_t frame_size = received_frame_size;
                received_frame_size = 0;
                if (frame_received){
                    frame_received(frame_size);
                }
            }
#endif
            if (wakeup_event){
                wakeup_event = 0;
                if (wakeup_handler){
                    wakeup_handler();
                }
            }
            break;
        default:
            break;
    }
}

static int btstack_uart_embedded_open(void){
    hal_uart_dma_init();
    hal_uart_dma_set_baud(uart_config->baudrate);

    // set up polling data_source
    btstack_run_loop_set_data_source_handler(&transport_data_source, &btstack_uart_embedded_process);
    btstack_run_loop_enable_data_source_callbacks(&transport_data_source, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_add_data_source(&transport_data_source);
    return 0;
} 

static int btstack_uart_embedded_close(void){

    // remove data source
    btstack_run_loop_disable_data_source_callbacks(&transport_data_source, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_remove_data_source(&transport_data_source);

    // close device
    // ...
    return 0;
}

static void btstack_uart_embedded_set_block_received( void (*block_handler)(void)){
    block_received = block_handler;
}

static void btstack_uart_embedded_set_block_sent( void (*block_handler)(void)){
    block_sent = block_handler;
}

#ifdef ENABLE_H5
static void btstack_uart_embedded_set_frame_received( void (*frame_handler)(uint16_t frame_size)){
    frame_received = frame_handler;
}

static void btstack_uart_embedded_set_frame_sent( void (*frame_handler)(void)){
    frame_sent = frame_handler;
}
#endif

static void btstack_uart_embedded_set_wakeup_handler( void (*the_wakeup_handler)(void)){
    wakeup_handler = the_wakeup_handler;
}

static int btstack_uart_embedded_set_parity(int parity){
    return 0;
}

static void btstack_uart_embedded_send_block(const uint8_t *data, uint16_t size){
    hal_uart_dma_send_block(data, size);
}

static void btstack_uart_embedded_receive_block(uint8_t *buffer, uint16_t len){
    hal_uart_dma_receive_block(buffer, len);
}

#ifdef ENABLE_H5
// -----------------------------
// SLIP ENCODING
static void btstack_uart_embedded_encode_chunk_and_send(void){
    uint16_t pos = 0;
    while (btstack_slip_encoder_has_data() & (pos < SLIP_TX_CHUNK_LEN)) {
        btstack_uart_embedded_slip_outgoing_buffer[pos++] = btstack_slip_encoder_get_byte();
    }

    // setup async write and start sending
    log_debug("slip: send %d bytes", pos);
    btstack_uart_embedded_send_block(btstack_uart_embedded_slip_outgoing_buffer, pos);
}

static void btstack_uart_embedded_send_frame(const uint8_t * frame, uint16_t frame_size){

    slip_outgoing_active = 1;

    // Prepare encoding of Header + Packet (+ DIC)
    btstack_slip_encoder_start(frame, frame_size);

    // Fill rest of chunk from packet and send
    btstack_uart_embedded_encode_chunk_and_send();
}
// SLIP ENCODING
// -----------------------------
static void btstack_uart_embedded_receive_frame(uint8_t *buffer, uint16_t len){
    hal_uart_dma_receive_frame(buffer, len);
}
#endif

static int btstack_uart_embedded_get_supported_sleep_modes(void){
#ifdef HAVE_HAL_UART_DMA_SLEEP_MODES
	return hal_uart_dma_get_supported_sleep_modes();
#else
	return BTSTACK_UART_SLEEP_MASK_RTS_HIGH_WAKE_ON_CTS_PULSE;
#endif
}

static void btstack_uart_embedded_set_sleep(btstack_uart_sleep_mode_t sleep_mode){
	log_info("set sleep %u", sleep_mode);
	if (sleep_mode == BTSTACK_UART_SLEEP_RTS_HIGH_WAKE_ON_CTS_PULSE){
		hal_uart_dma_set_csr_irq_handler(&btstack_uart_embedded_cts_pulse);
	} else {
		hal_uart_dma_set_csr_irq_handler(NULL);
	}

#ifdef HAVE_HAL_UART_DMA_SLEEP_MODES
	hal_uart_dma_set_sleep_mode(sleep_mode);
#else
	hal_uart_dma_set_sleep(sleep_mode != BTSTACK_UART_SLEEP_OFF);
#endif
	log_info("done");
}

static const btstack_uart_t btstack_uart_embedded = {
    /* int  (*init)(hci_transport_config_uart_t * config); */         &btstack_uart_embedded_init,
    /* int  (*open)(void); */                                         &btstack_uart_embedded_open,
    /* int  (*close)(void); */                                        &btstack_uart_embedded_close,
    /* void (*set_block_received)(void (*handler)(void)); */          &btstack_uart_embedded_set_block_received,
    /* void (*set_block_sent)(void (*handler)(void)); */              &btstack_uart_embedded_set_block_sent,
#ifdef ENABLE_H5
    /* void (*set_frame_received)(void (*handler)(..); */             &btstack_uart_embedded_set_frame_received,
    /* void (*set_frame_sent)(void (*handler)(void)); */              &btstack_uart_embedded_set_frame_sent,
#endif
    /* int  (*set_baudrate)(uint32_t baudrate); */                    &hal_uart_dma_set_baud,
    /* int  (*set_parity)(int parity); */                             &btstack_uart_embedded_set_parity,
#ifdef HAVE_UART_DMA_SET_FLOWCONTROL
    /* int  (*set_flowcontrol)(int flowcontrol); */                   &hal_uart_dma_set_flowcontrol,
#else
    /* int  (*set_flowcontrol)(int flowcontrol); */                   NULL,
#endif
    /* void (*receive_block)(uint8_t *buffer, uint16_t len); */       &btstack_uart_embedded_receive_block,
    /* void (*send_block)(const uint8_t *buffer, uint16_t length); */ &btstack_uart_embedded_send_block,    
#ifdef ENABLE_H5
    /* void (*receive_block)(uint8_t *buffer, uint16_t len); */       &btstack_uart_embedded_receive_frame,
    /* void (*send_block)(const uint8_t *buffer, uint16_t length); */ &btstack_uart_embedded_send_frame,    
#endif
	/* int (*get_supported_sleep_modes); */                           &btstack_uart_embedded_get_supported_sleep_modes,
    /* void (*set_sleep)(btstack_uart_sleep_mode_t sleep_mode); */    &btstack_uart_embedded_set_sleep,
    /* void (*set_wakeup_handler)(void (*handler)(void)); */          &btstack_uart_embedded_set_wakeup_handler,
};

const btstack_uart_t * btstack_uart_embedded_instance(void){
	return &btstack_uart_embedded;
}
