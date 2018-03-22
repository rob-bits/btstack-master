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

#define __BTSTACK_FILE__ "le_streamer_and_counter_client.c"

// *****************************************************************************
/* EXAMPLE_START(le_streamer): LE Streamer - Stream data over GATT.
 *
 * @text All newer operating systems provide GATT Client functionality.
 * This example shows how to get a maximal throughput via BLE:
 * - send whenever possible,
 * - use the max ATT MTU.
 *
 * @text In theory, we should also update the connection parameters, but we already get
 * a connection interval of 30 ms and there's no public way to use a shorter 
 * interval with iOS (if we're not implementing an HID device).
 *
 * @text Note: To start the streaming, run the example.
 * On remote device use some GATT Explorer, e.g. LightBlue, BLExplr to enable notifications.
 */
 // *****************************************************************************

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btstack.h"
#include "le_streamer.h"

#define REPORT_INTERVAL_MS 3000
#define MAX_NR_CONNECTIONS 3 

static void  packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static int   att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
static void  streamer(void);

const uint8_t adv_data[] = {
    // Flags general discoverable, BR/EDR not supported
    0x02, 0x01, 0x06, 
    // Name
    0x0c, 0x09, 'L', 'E', ' ', 'S', 't', 'r', 'e', 'a', 'm', 'e', 'r', 
};
const uint8_t adv_data_len = sizeof(adv_data);

static btstack_packet_callback_registration_t hci_event_callback_registration;

typedef enum {
    TC_IDLE,
    TC_W4_SCAN_RESULT,
    TC_W4_CONNECT,
    TC_W4_SERVICE_RESULT,
    TC_W4_CHARACTERISTIC_RESULT,
    TC_W4_SUBSCRIBED
} gc_state_t;

static gc_state_t state = TC_IDLE;

// support for multiple clients
typedef struct {
    char name;
    int le_notification_enabled;
    uint16_t value_handle;
    hci_con_handle_t connection_handle;
    int  counter;
    char test_data[200];
    int  test_data_len;
    uint32_t test_data_sent;
    uint32_t test_data_received;
    uint32_t test_data_start;
} le_streamer_connection_t;
static le_streamer_connection_t le_streamer_connections[MAX_NR_CONNECTIONS];

// round robin sending
static int connection_index;

// addr and type of device with correct name
static bd_addr_t      le_counter_addr;
static bd_addr_type_t le_counter_addr_type;

static hci_con_handle_t connection_handle;

// On the GATT Server, RX Characteristic is used for receive data via Write, and TX Characteristic is used to send data via Notifications
static uint8_t le_counter_service_uuid[16]        = { 0x00, 0x00, 0xFF, 0x10, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
static uint8_t le_counter_characteristic_uuid[16] = { 0x00, 0x00, 0xFF, 0x11, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};

static gatt_client_service_t le_counter_service;
static gatt_client_characteristic_t le_counter_characteristic;

static gatt_client_notification_t notification_listener;
static int listener_registered;

static void init_connections(void){
    // track connections
    int i;
    for (i=0;i<MAX_NR_CONNECTIONS;i++){
        le_streamer_connections[i].connection_handle = HCI_CON_HANDLE_INVALID;
        le_streamer_connections[i].name = 'A' + i;
    }
}

static le_streamer_connection_t * connection_for_conn_handle(hci_con_handle_t conn_handle){
    int i;
    for (i=0;i<MAX_NR_CONNECTIONS;i++){
        if (le_streamer_connections[i].connection_handle == conn_handle) return &le_streamer_connections[i];
    }
    return NULL;
}

static void next_connection_index(void){
    connection_index++;
    if (connection_index == MAX_NR_CONNECTIONS){
        connection_index = 0;
    }
}

/* @section Main Application Setup
 *
 * @text Listing MainConfiguration shows main application code.
 * It initializes L2CAP, the Security Manager, and configures the ATT Server with the pre-compiled
 * ATT Database generated from $le_streamer.gatt$. Finally, it configures the advertisements 
 * and boots the Bluetooth stack. 
 */
 
/* LISTING_START(MainConfiguration): Init L2CAP, SM, ATT Server, and enable advertisements */

static void le_streamer_setup(void){

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

    gatt_client_init();

    // setup le device db
    le_device_db_init();

    // setup SM: Display only
    sm_init();

    // setup ATT server
    att_server_init(profile_data, NULL, att_write_callback);    
    att_server_register_packet_handler(packet_handler);
    
    // setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
    gap_advertisements_enable(1);

    // use different connection parameters: conn interval min/max (* 1.25 ms), slave latency, supervision timeout, CE len min/max (* 0.6125 ms) 
    gap_set_connection_parameters(0x60, 0x30, 0x60, 0x60, 0, 1000, 0x01, 0x18 * 2);

    // init client state
    init_connections();
}
/* LISTING_END */

/*
 * @section Track throughput
 * @text We calculate the throughput by setting a start time and measuring the amount of 
 * data sent. After a configurable REPORT_INTERVAL_MS, we print the throughput in kB/s
 * and reset the counter and start time.
 */

/* LISTING_START(tracking): Tracking throughput */

static void test_reset(le_streamer_connection_t * context){
    context->test_data_start = btstack_run_loop_get_time_ms();
    context->test_data_sent = 0;
}

static void test_report(le_streamer_connection_t * context){
    // evaluate
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t time_passed = now - context->test_data_start;
    if (time_passed < REPORT_INTERVAL_MS) return;
    // print speed
    int bytes_per_second = (context->test_data_sent  + context->test_data_received) * 1000 / time_passed;
    printf("%c: %"PRIu32" bytes sent + %"PRIu32" bytes received -> %u.%03u kB/s\n", 
        context->name, context->test_data_sent, context->test_data_received, 
        bytes_per_second / 1000, bytes_per_second % 1000);

    // restart
    context->test_data_start     = now;
    context->test_data_sent      = 0;
    context->test_data_received  = 0;
}

static void test_track_sent(le_streamer_connection_t * context, int bytes_sent){
    context->test_data_sent += bytes_sent;
    test_report(context);
}

static void test_track_received(le_streamer_connection_t * context, int bytes_received){
    context->test_data_received += bytes_received;
    test_report(context);
}

/* LISTING_END(tracking): Tracking throughput */

// returns 1 if name is found in advertisement
static int advertisement_report_contains_name(const char * name, uint8_t * advertisement_report){
    // get advertisement from report event
    const uint8_t * _adv_data = gap_event_advertising_report_get_data(advertisement_report);
    uint16_t        adv_len  = gap_event_advertising_report_get_data_length(advertisement_report);
    int             name_len = strlen(name);

    // iterate over advertisement data
    ad_context_t context;
    for (ad_iterator_init(&context, adv_len, _adv_data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
        uint8_t data_type    = ad_iterator_get_data_type(&context);
        uint8_t data_size    = ad_iterator_get_data_len(&context);
        const uint8_t * data = ad_iterator_get_data(&context);
        int i;
        int match = 1;
        switch (data_type){
            case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
                // compare common prefix
                for (i=0; i<data_size && i<name_len;i++){
                    if (data[i] != name[i]) {
                        match = 0;
                        break;
                    }
                }
                if (match) return 1;
                break;
            default:
                break;
        }
    }
    return 0;
}

static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    int status;
    char message[30];

    switch(state){
        case TC_W4_SERVICE_RESULT:
            switch(hci_event_packet_get_type(packet)){
                case GATT_EVENT_SERVICE_QUERY_RESULT:
                    gatt_event_service_query_result_get_service(packet, &le_counter_service);
                    break;
                case GATT_EVENT_QUERY_COMPLETE:
                    if (packet[4] != 0){
                        printf("SERVICE_QUERY_RESULT - Error status %x.\n", packet[4]);
                        gap_disconnect(connection_handle);
                        break;  
                    } 
                    state = TC_W4_CHARACTERISTIC_RESULT;
                    printf("\nSearch for counter characteristic.\n");
                    gatt_client_discover_characteristics_for_service_by_uuid128(handle_gatt_client_event, connection_handle, &le_counter_service, le_counter_characteristic_uuid);
                    break;
                default:
                    break;
            }
            break;
            
        case TC_W4_CHARACTERISTIC_RESULT:
            switch(hci_event_packet_get_type(packet)){
                case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
                    gatt_event_characteristic_query_result_get_characteristic(packet, &le_counter_characteristic);
                    break;
                case GATT_EVENT_QUERY_COMPLETE:
                    if (packet[4] != 0){
                        printf("CHARACTERISTIC_QUERY_RESULT - Error status %x.\n", packet[4]);
                        gap_disconnect(connection_handle);
                        break;  
                    } 
                    state = TC_W4_SUBSCRIBED;
                    printf("\nConfigure counter for notify.\n");
                    status = gatt_client_write_client_characteristic_configuration(handle_gatt_client_event, connection_handle, &le_counter_characteristic, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                    break;
                default:
                    break;
            }
            break;
        case TC_W4_SUBSCRIBED:
            switch(hci_event_packet_get_type(packet)){
                case GATT_EVENT_NOTIFICATION:
                    memset(message, 0, sizeof(message));
                    memcpy(message, gatt_event_notification_get_value(packet), gatt_event_notification_get_value_length(packet));
                    printf("Counter: %s\n", message);
                    break;
                case GATT_EVENT_QUERY_COMPLETE:
                    // register handler for notifications
                    listener_registered = 1;
                    gatt_client_listen_for_characteristic_value_updates(&notification_listener, handle_gatt_client_event, connection_handle, &le_counter_characteristic);
                    break;
                default:
                    printf("Unknown packet type %x\n", hci_event_packet_get_type(packet));
                    break;
            }
            break;

        default:
            printf("error\n");
            break;
    }
    
}

/* 
 * @section Packet Handler
 *
 * @text The packet handler is used to stop the notifications and reset the MTU on connect
 * It would also be a good place to request the connection parameter update as indicated 
 * in the commented code block.
 */

static void start_scanning(void){
    state = TC_W4_SCAN_RESULT;
    gap_set_scan_parameters(0,0x0010, 0x060);
    gap_start_scan();
}

/* LISTING_START(packetHandler): Packet Handler */
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    
    int mtu;
    uint16_t conn_interval;
    hci_con_handle_t con_handle;
    le_streamer_connection_t * context;
    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case BTSTACK_EVENT_STATE:
                    // BTstack activated, get started
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        printf("To start the streaming, please run the le_streamer_client example on other device, or use some GATT Explorer, e.g. LightBlue, BLExplr.\n");
                        printf("Start scanning for LE Counter!\n");
                        start_scanning();
                    } 
                    break;
                case GAP_EVENT_ADVERTISING_REPORT:
                    if (state != TC_W4_SCAN_RESULT) return;
                    // check name in advertisement
                    if (!advertisement_report_contains_name("LE Counter", packet)) return;
                    // store address and type
                    gap_event_advertising_report_get_address(packet, le_counter_addr);
                    le_counter_addr_type = gap_event_advertising_report_get_address_type(packet);
                    // stop scanning, and connect to the device
                    state = TC_W4_CONNECT;
                    gap_stop_scan();
                    printf("Stop scan. Connect to device with addr %s.\n", bd_addr_to_str(le_counter_addr));
                    gap_connect(le_counter_addr,le_counter_addr_type);
                    break;
                case HCI_EVENT_DISCONNECTION_COMPLETE:
                    con_handle = hci_event_disconnection_complete_get_connection_handle(packet);
                    if (con_handle == connection_handle){
                        // LE Counter
                        printf("Counter: Disconnect, reason %02x\n", hci_event_disconnection_complete_get_reason(packet));
                        connection_handle = HCI_CON_HANDLE_INVALID;
                        start_scanning();
                        break;
                    }
                    context = connection_for_conn_handle(hci_event_disconnection_complete_get_connection_handle(packet));
                    if (context){
                        // LE Streamer
                        printf("%c: Disconnect, reason %02x\n", context->name, hci_event_disconnection_complete_get_reason(packet));                    
                        context->le_notification_enabled = 0;
                        context->connection_handle = HCI_CON_HANDLE_INVALID;
                    }
                    if (!context) break;
                    break;
                case HCI_EVENT_LE_META:
                    switch (hci_event_le_meta_get_subevent_code(packet)) {
                        case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                            // setup new
                            printf("Connected, role %u\n", hci_subevent_le_connection_complete_get_role(packet));
                            if (hci_subevent_le_connection_complete_get_role(packet) == HCI_ROLE_MASTER){
                                connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                                // initialize gatt client context with handle, and add it to the list of active clients
                                // query primary services
                                printf("\nSearch for LE Counter service.\n");
                                state = TC_W4_SERVICE_RESULT;
                                gatt_client_discover_primary_services_by_uuid128(handle_gatt_client_event, connection_handle, le_counter_service_uuid);
                            } else {
                                context = connection_for_conn_handle(HCI_CON_HANDLE_INVALID);
                                if (!context) break;
                                context->counter = 'A';
                                context->test_data_len = ATT_DEFAULT_MTU - 3;
                                context->connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                                // print connection parameters (without using float operations)
                                conn_interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
                                printf("%c: Connection Interval: %u.%02u ms\n", context->name, conn_interval * 125 / 100, 25 * (conn_interval & 3));
                                printf("%c: Connection Latency: %u\n", context->name, hci_subevent_le_connection_complete_get_conn_latency(packet));
                                // min con interval 20 ms 
                                // gap_request_connection_parameter_update(connection_handle, 0x10, 0x18, 0, 0x0048);
                                // printf("Connected, requesting conn param update for handle 0x%04x\n", connection_handle);
                            }
                            break;
                        default:
                            break;
                    }
                    break;  
                case ATT_EVENT_MTU_EXCHANGE_COMPLETE:
                    mtu = att_event_mtu_exchange_complete_get_MTU(packet) - 3;
                    context = connection_for_conn_handle(att_event_mtu_exchange_complete_get_handle(packet));
                    if (!context) break;
                    context->test_data_len = btstack_min(mtu - 3, sizeof(context->test_data));
                    printf("%c: ATT MTU = %u => use test data of len %u\n", context->name, mtu, context->test_data_len);
                    break;
                case ATT_EVENT_CAN_SEND_NOW:
                    streamer();
                    break;
            }
    }
}

/* LISTING_END */
/*
 * @section Streamer
 *
 * @text The streamer function checks if notifications are enabled and if a notification can be sent now.
 * It creates some test data - a single letter that gets increased every time - and tracks the data sent.
 */

 /* LISTING_START(streamer): Streaming code */
static void streamer(void){

    // find next active streaming connection
    int old_connection_index = connection_index;
    while (1){
        // active found?
        if ((le_streamer_connections[connection_index].connection_handle != HCI_CON_HANDLE_INVALID) &&
            (le_streamer_connections[connection_index].le_notification_enabled)) break;
        
        // check next
        next_connection_index();

        // none found
        if (connection_index == old_connection_index) return;
    }

    le_streamer_connection_t * context = &le_streamer_connections[connection_index];

    // create test data
    context->counter++;
    if (context->counter > 'Z') context->counter = 'A';
    memset(context->test_data, context->counter, context->test_data_len);

    // send
    att_server_notify(context->connection_handle, context->value_handle, (uint8_t*) context->test_data, context->test_data_len);

    // track
    test_track_sent(context, context->test_data_len);

    // request next send event
    att_server_request_can_send_now_event(context->connection_handle);

    // check next
    next_connection_index();
} 
/* LISTING_END */

/*
 * @section ATT Write
 *
 * @text The only valid ATT write in this example is to the Client Characteristic Configuration, which configures notification
 * and indication. If the ATT handle matches the client configuration handle, the new configuration value is stored.
 * If notifications get enabled, an ATT_EVENT_CAN_SEND_NOW is requested. See Listing attWrite.
 */

/* LISTING_START(attWrite): ATT Write */
static int att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    UNUSED(offset);

    // printf("att_write_callback att_handle %04x, transaction mode %u\n", att_handle, transaction_mode);
    if (transaction_mode != ATT_TRANSACTION_MODE_NONE) return 0;
    le_streamer_connection_t * context = connection_for_conn_handle(con_handle);
    switch(att_handle){
        case ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE:
        case ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE:
            context->le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
            printf("%c: Notifications enabled %u\n", context->name, context->le_notification_enabled); 
            if (context->le_notification_enabled){
                switch (att_handle){
                    case ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE:
                        context->value_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE;
                        break;
                    case ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE:
                        context->value_handle = ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE;
                        break;
                    default:
                        break;
                }
                att_server_request_can_send_now_event(context->connection_handle);
            }
            test_reset(context);
            break;
        case ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE:
        case ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE:
            test_track_received(context, buffer_size);
            break;
        default:
            printf("Write to 0x%04x, len %u\n", att_handle, buffer_size);
    }
    return 0;
}
/* LISTING_END */

int btstack_main(void);
int btstack_main(void)
{
    le_streamer_setup();

    // turn on!
	hci_power_control(HCI_POWER_ON);
	    
    return 0;
}
/* EXAMPLE_END */
