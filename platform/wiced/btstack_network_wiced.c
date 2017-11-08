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

#define __BTSTACK_FILE__ "btstack_network_wiced.c"

/*
 * btstack_network_wiced.c
 */


#include "wiced.h"
#include "platform/wwd_platform_interface.h"
#include "platform_ethernet.h"
#include "network/wwd_buffer_interface.h"

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_network.h"
#include "btstack_run_loop_wiced.h"

// hack to set virtual hw address
extern uint8_t wwd_ethernet_hw_address[6];

static void (*btstack_network_send_packet_callback)(const uint8_t * packet, uint16_t size);

static uint8_t platform_ethernet_initialized = WICED_FALSE;

// outgoing queue & next buffer protected by mutex
static wiced_mutex_t         platform_ethernet_outgoing_queue_mutex;
static wiced_buffer_t        platform_ethernet_next_bnep_packet;
static btstack_linked_list_t platform_ethernet_outgoing_queue;

// platform ethernet implementation
platform_result_t platform_ethernet_deinit                ( void ){
    log_info("platform_ethernet_deinit");
    platform_ethernet_initialized = WICED_FALSE;
    return PLATFORM_SUCCESS;
}
wiced_bool_t      platform_ethernet_is_inited             ( void ){
    log_info("platform_ethernet_is_inited");
    return platform_ethernet_initialized;
}
platform_result_t platform_ethernet_start                 ( void ){
    log_info("platform_ethernet_start");
    return PLATFORM_SUCCESS;
}
platform_result_t platform_ethernet_stop                  ( void ){
    log_info("platform_ethernet_stop");
    return PLATFORM_SUCCESS;
}
platform_result_t platform_ethernet_get_config            ( platform_ethernet_config_t** config ){
    log_info("platform_ethernet_get_config");
    return PLATFORM_SUCCESS;
}
wiced_bool_t platform_ethernet_is_ready_to_transceive( void ){
    log_info("platform_ethernet_is_ready_to_transceive");
    return WICED_TRUE;
}
platform_result_t platform_ethernet_set_loopback_mode     ( platform_ethernet_loopback_mode_t loopback_mode ){
    log_info("platform_ethernet_set_loopback_mode");
    return PLATFORM_SUCCESS;
}

//

static wiced_result_t platform_ethernet_outgoing_notify(void * arg){
    UNUSED(arg);
    log_debug("platform_ethernet_outgoing_notify %p", platform_ethernet_next_bnep_packet);
    if (platform_ethernet_next_bnep_packet){
        (*btstack_network_send_packet_callback)(platform_ethernet_next_bnep_packet->payload, platform_ethernet_next_bnep_packet->len);
    } else {
        log_error("platform_ethernet_next_bnep_packet == nil!");
    }
    return WICED_SUCCESS;
}

static void platform_ethernet_outgoing_process(void){
    // get next packet if none waiting
    int notify_callback = 0;
    wiced_rtos_lock_mutex(&platform_ethernet_outgoing_queue_mutex);
    if (!platform_ethernet_next_bnep_packet){
        platform_ethernet_next_bnep_packet = (wiced_buffer_t) btstack_linked_list_pop(&platform_ethernet_outgoing_queue);
        notify_callback = platform_ethernet_next_bnep_packet != NULL;
    } 
    wiced_rtos_unlock_mutex(&platform_ethernet_outgoing_queue_mutex);

    log_debug("platform_ethernet_outgoing_process next packet %p", platform_ethernet_next_bnep_packet);

    // notify callback if we got a new packet
    if (notify_callback){
        btstack_run_loop_wiced_execute_code_on_main_thread(&platform_ethernet_outgoing_notify, NULL);
    }    
}

/** 
 * @brief Notify network interface that packet from send_packet_callback was sent and the next packet can be delivered.
 */
void btstack_network_packet_sent(void){

    // free current packet
    wiced_buffer_t buffer_to_release = NULL;
    wiced_rtos_lock_mutex(&platform_ethernet_outgoing_queue_mutex);
    if (platform_ethernet_next_bnep_packet) {
        buffer_to_release = platform_ethernet_next_bnep_packet;
        platform_ethernet_next_bnep_packet = NULL;
    }
    wiced_rtos_unlock_mutex(&platform_ethernet_outgoing_queue_mutex);

    if (buffer_to_release){
         pbuf_free(buffer_to_release);
    }   

    // trigger next
    platform_ethernet_outgoing_process();
}

// called by network stack to send packet
// put in queue.
// trigger send next if not already done so
platform_result_t platform_ethernet_send_data             ( wiced_buffer_t buffer ){
    log_debug("platform_ethernet_send_data %p", buffer);
    wiced_rtos_lock_mutex(&platform_ethernet_outgoing_queue_mutex);
    btstack_linked_list_add_tail(&platform_ethernet_outgoing_queue, (btstack_linked_item_t*) buffer);
    wiced_rtos_unlock_mutex(&platform_ethernet_outgoing_queue_mutex);
    platform_ethernet_outgoing_process();
    return PLATFORM_SUCCESS;
}

platform_result_t platform_ethernet_init                  ( void ){
    log_info("platform_ethernet_init");
    if (!platform_ethernet_initialized) {
        wiced_rtos_init_mutex(&platform_ethernet_outgoing_queue_mutex);
        platform_ethernet_initialized = WICED_TRUE;
    }
    return PLATFORM_SUCCESS;
}

/**
 * @brief Initialize network interface
 * @param send_packet_callback
 */
void btstack_network_init(void (*send_packet_callback)(const uint8_t * packet, uint16_t size)){
    btstack_network_send_packet_callback = send_packet_callback;
    wiced_network_init();
}

int btstack_network_up(bd_addr_t network_address){
    log_info("btstack_network_up start addr %s", bd_addr_to_str(network_address));
    memcpy(wwd_ethernet_hw_address, network_address, 6);
    platform_ethernet_init();
    return 0;
}


/**
 * @brief Get network name after network was activated
 * @note e.g. tapX on Linux, might not be useful on all platforms
 * @returns network name
 */
const char * btstack_network_get_name(void){
    return "bt";
}

/**
 * @brief Bring up network interfacd
 * @param network_address
 * @return 0 if ok
 */
int btstack_network_down(void){
    log_info("btstack_network_up down");
    platform_ethernet_deinit();
    return 0;
}

/** 
 * @brief Receive packet on network interface, e.g., forward packet to TCP/IP stack 
 * @param packet
 * @param size
 */
void btstack_network_process_packet(const uint8_t * packet, uint16_t size){
    wiced_buffer_t buffer;
    host_buffer_get(&buffer, WWD_NETWORK_RX, size, WICED_TRUE);
    memcpy(buffer->payload, packet, size);
    host_network_process_ethernet_data(buffer, WWD_ETHERNET_INTERFACE);
}

