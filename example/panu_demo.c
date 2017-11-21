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

#define __BTSTACK_FILE__ "panu_demo.c"

/*
 * panu_demo.c
 * Author: Ole Reinhardt <ole.reinhardt@kernelconcepts.de>
 */

/* EXAMPLE_START(panu_demo): PANU Demo
 *
 * @text This example implements both a PANU client and a server. In server mode, it 
 * sets up a BNEP server and registers a PANU SDP record and waits for incoming connections.
 * In client mode, it connects to a remote device, does an SDP Query to identify the PANU
 * service and initiates a BNEP connection.
 *
 * Note: currently supported only on Linux and Mac.
 */

#include <stdio.h>

#include "btstack_config.h"
#include "btstack.h"

#define AUTO_RECONNECT
#define AUTO_RECONNECT_DELAY_MS 5000

static uint8_t invalid_addr[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static int record_id = -1;
static uint16_t bnep_l2cap_psm      = 0;
static uint32_t bnep_remote_uuid    = 0;
static uint16_t bnep_version        = 0;
static uint16_t bnep_cid            = 0;

static uint16_t sdp_bnep_l2cap_psm      = 0;
static uint16_t sdp_bnep_version        = 0;
static uint32_t sdp_bnep_remote_uuid    = 0;

static uint8_t   attribute_value[1000];
static const unsigned int attribute_value_buffer_size = sizeof(attribute_value);

// dummy HID service to allow iPhone to connect
static uint8_t hid_service_buffer[250];
static const char hid_device_name[] = "Dummy Keyboard";

// from USB HID Specification 1.1, Appendix B.1
const uint8_t hid_descriptor_dummy[] = {
    0x05, 0x01,                    // Usage Page (Generic Desktop)
    0x09, 0x06,                    // Usage (Keyboard)
};

// default: pairint mode = discoverable, wait for pairing/incoming HID connection
static int pairing_mode = 1;

static bd_addr_t remote_addr;

static btstack_packet_callback_registration_t hci_event_callback_registration;

static btstack_timer_source_t reconnect_timer;

// outgoing network packet
static const uint8_t * network_buffer;
static uint16_t network_buffer_len;

/* @section Main application configuration
 *
 * @text In the application configuration, L2CAP and BNEP are initialized and a BNEP service, for server mode,
 * is registered, before the Bluetooth stack gets started, as shown in Listing PanuSetup.
 */

/* LISTING_START(PanuSetup): Panu setup */
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_sdp_client_query_result(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void network_send_packet_callback(const uint8_t * packet, uint16_t size);

static void panu_setup(void){

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Initialize L2CAP 
    l2cap_init();

    // Initialise BNEP
    bnep_init();

    // Minimum L2CAP MTU for bnep is 1691 bytes
    bnep_register_service(packet_handler, BLUETOOTH_SERVICE_CLASS_PANU, 1691);  

    // Register dummy HID service
    sdp_init();
    memset(hid_service_buffer, 0, sizeof(hid_service_buffer));
    // hid sevice subclass 2540 Keyboard, hid counntry code 33 US, hid virtual cable off, hid reconnect initiate off, hid boot device off 
    hid_create_sdp_record(hid_service_buffer, 0x10001, 0x2540, 33, 0, 0, 0, hid_descriptor_dummy, sizeof(hid_descriptor_dummy), hid_device_name);
    printf("HID service record size: %u\n", de_get_len( hid_service_buffer));
    sdp_register_service(hid_service_buffer);

    // HID Device
    hid_device_init();
    hid_device_register_packet_handler(&packet_handler);

    if (pairing_mode){
        // Discoverable
        // Set local name with a template Bluetooth address, that will be automatically
        // replaced with a actual address once it is available, i.e. when BTstack boots
        // up and starts talking to a Bluetooth module.
        gap_set_local_name("PANU Demo 00:00:00:00:00:00");
        gap_discoverable_control(1);
        gap_set_class_of_device(0x2540);
    }

    // Initialize network interface
    btstack_network_init(&network_send_packet_callback);
}
/* LISTING_END */

// PANU client routines 
static char * get_string_from_data_element(uint8_t * element){
    de_size_t de_size = de_get_size_type(element);
    int pos     = de_get_header_size(element);
    int len = 0;
    switch (de_size){
        case DE_SIZE_VAR_8:
            len = element[1];
            break;
        case DE_SIZE_VAR_16:
            len = big_endian_read_16(element, 1);
            break;
        default:
            break;
    }
    char * str = (char*)malloc(len+1);
    memcpy(str, &element[pos], len);
    str[len] ='\0';
    return str; 
}

static void start_outgoing_connection(btstack_timer_source_t * ts){
    UNUSED(ts);
    // reconnect
    printf("Start SDP BNEP query for remote PAN Network Access Point (NAP).\n");
    bnep_l2cap_psm = 0;
    bnep_remote_uuid = 0;
    sdp_client_query_uuid16(&handle_sdp_client_query_result, remote_addr, BLUETOOTH_SERVICE_CLASS_NAP);
}

static void schedule_outgoing_connection(void){
    printf("Outgoing connection scheduled\n");
    // set one-shot timer
    reconnect_timer.process = &start_outgoing_connection;
    btstack_run_loop_set_timer(&reconnect_timer, AUTO_RECONNECT_DELAY_MS);
    btstack_run_loop_add_timer(&reconnect_timer);
}

/* @section SDP parser callback 
 * 
 * @text The SDP parsers retrieves the BNEP PAN UUID as explained in  
 * Section [on SDP BNEP Query example](#sec:sdpbnepqueryExample}.
 */
static void handle_sdp_client_record_complete(void){

    printf("SDP BNEP Record complete\n");

    // accept first entry or if we foudn a NAP and only have a PANU yet
    if ((bnep_remote_uuid == 0) || (sdp_bnep_remote_uuid == BLUETOOTH_SERVICE_CLASS_NAP ||  bnep_remote_uuid == BLUETOOTH_SERVICE_CLASS_PANU)){
        bnep_l2cap_psm   = sdp_bnep_l2cap_psm;
        bnep_remote_uuid = sdp_bnep_remote_uuid;
        bnep_version     = sdp_bnep_version;
    }
}

static void handle_sdp_client_query_result(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {

    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    des_iterator_t des_list_it;
    des_iterator_t prot_it;
    char *str;

    switch (hci_event_packet_get_type(packet)){
        case SDP_EVENT_QUERY_ATTRIBUTE_VALUE:
            // Handle new SDP record 
            if (sdp_event_query_attribute_byte_get_record_id(packet) != record_id) {
                handle_sdp_client_record_complete();
                // next record started
                record_id = sdp_event_query_attribute_byte_get_record_id(packet);
                printf("SDP Record: Nr: %d\n", record_id);
            }

            if (sdp_event_query_attribute_byte_get_attribute_length(packet) <= attribute_value_buffer_size) {
                attribute_value[sdp_event_query_attribute_byte_get_data_offset(packet)] = sdp_event_query_attribute_byte_get_data(packet);
                
                if ((uint16_t)(sdp_event_query_attribute_byte_get_data_offset(packet)+1) == sdp_event_query_attribute_byte_get_attribute_length(packet)) {

                    switch(sdp_event_query_attribute_byte_get_attribute_id(packet)) {
                        case BLUETOOTH_ATTRIBUTE_SERVICE_CLASS_ID_LIST:
                            if (de_get_element_type(attribute_value) != DE_DES) break;
                            for (des_iterator_init(&des_list_it, attribute_value); des_iterator_has_more(&des_list_it); des_iterator_next(&des_list_it)) {
                                uint8_t * element = des_iterator_get_element(&des_list_it);
                                if (de_get_element_type(element) != DE_UUID) continue;
                                uint32_t uuid = de_get_uuid32(element);
                                switch (uuid){
                                    case BLUETOOTH_SERVICE_CLASS_PANU:
                                    case BLUETOOTH_SERVICE_CLASS_NAP:
                                    case BLUETOOTH_SERVICE_CLASS_GN:
                                        printf("SDP Attribute 0x%04x: BNEP PAN protocol UUID: %04x\n", sdp_event_query_attribute_byte_get_attribute_id(packet), (int) uuid);
                                        sdp_bnep_remote_uuid = uuid;
                                        break;
                                    default:
                                        break;
                                }
                            }
                            break;
                        case 0x0100:
                        case 0x0101:
                            str = get_string_from_data_element(attribute_value);
                            printf("SDP Attribute: 0x%04x: %s\n", sdp_event_query_attribute_byte_get_attribute_id(packet), str);
                            free(str);
                            break;
                        case BLUETOOTH_ATTRIBUTE_PROTOCOL_DESCRIPTOR_LIST: {
                                printf("SDP Attribute: 0x%04x\n", sdp_event_query_attribute_byte_get_attribute_id(packet));

                                for (des_iterator_init(&des_list_it, attribute_value); des_iterator_has_more(&des_list_it); des_iterator_next(&des_list_it)) {                                    
                                    uint8_t       *des_element;
                                    uint8_t       *element;
                                    uint32_t       uuid;

                                    if (des_iterator_get_type(&des_list_it) != DE_DES) continue;

                                    des_element = des_iterator_get_element(&des_list_it);
                                    des_iterator_init(&prot_it, des_element);
                                    element = des_iterator_get_element(&prot_it);
                                    
                                    if (de_get_element_type(element) != DE_UUID) continue;
                                    
                                    uuid = de_get_uuid32(element);
                                    switch (uuid){
                                        case BLUETOOTH_PROTOCOL_L2CAP:
                                            if (!des_iterator_has_more(&prot_it)) continue;
                                            des_iterator_next(&prot_it);
                                            de_element_get_uint16(des_iterator_get_element(&prot_it), &sdp_bnep_l2cap_psm);
                                            break;
                                        case BLUETOOTH_PROTOCOL_BNEP:
                                            if (!des_iterator_has_more(&prot_it)) continue;
                                            des_iterator_next(&prot_it);
                                            de_element_get_uint16(des_iterator_get_element(&prot_it), &sdp_bnep_version);
                                            break;
                                        default:
                                            break;
                                    }
                                }
                                printf("Summary: uuid 0x%04x, l2cap_psm 0x%04x, bnep_version 0x%04x\n", (int) sdp_bnep_remote_uuid, sdp_bnep_l2cap_psm, sdp_bnep_version);

                            }
                            break;
                        default:
                            break;
                    }
                }
            } else {
                fprintf(stderr, "SDP attribute value buffer size exceeded: available %d, required %d\n", attribute_value_buffer_size, sdp_event_query_attribute_byte_get_attribute_length(packet));
            }
            break;
            
        case SDP_EVENT_QUERY_COMPLETE:
            if (sdp_event_query_complete_get_status(packet)){
                fprintf(stderr, "SDP query failed with status 0x%02x\n", sdp_event_query_complete_get_status(packet));
#ifdef AUTO_RECONNECT
                schedule_outgoing_connection();
#endif
                break;
            }

            handle_sdp_client_record_complete();
            fprintf(stderr, "General query done bnep psm %04x.\n", bnep_l2cap_psm);
            if (bnep_l2cap_psm){
                /* Create BNEP connection */
                bnep_connect(packet_handler, remote_addr, bnep_l2cap_psm, BLUETOOTH_SERVICE_CLASS_PANU, bnep_remote_uuid);
            } else {
                fprintf(stderr, "No BNEP service found\n");
                #ifdef AUTO_RECONNECT
                                schedule_outgoing_connection();
                #endif
            }
            break;
    }
}

#ifdef WICED_VERSION
// Demo code for WICED
// - start DHCP (blocking...)
// - ping...
#include "wiced.h"
#include "btstack_link_key_db_wiced_dct.h"
#include "le_device_db_wiced_dct.h"

extern wiced_result_t wiced_ip_up( wiced_interface_t interface, wiced_network_config_t config, const wiced_ip_setting_t* ip_settings );

static wiced_thread_t    panu_demo_wiced_thread;
static wiced_semaphore_t panu_demo_wiced_semaphore;

static int network_up;

#define PING_TIMEOUT_MS          2000
#define PING_INTERVAL_MS         1000

static void panu_demo_wiced(wiced_thread_arg_t arg){
    UNUSED(arg);

    wiced_result_t status;

    while (1){
        printf("PANU DEMO WICED: wait for network up\n");
        wiced_rtos_get_semaphore(&panu_demo_wiced_semaphore, WICED_NEVER_TIMEOUT);

        printf("Network up, start DHCP\n");
        status = wiced_ip_up( WICED_ETHERNET_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
        printf("DHCP Done, result %u, Ping DEMO\n", status);
        
        uint32_t elapsed_ms;
        wiced_ip_address_t ip_address;
        SET_IPV4_ADDRESS(ip_address, 0x08080808);   // google dns

        while (network_up){
            status = wiced_ping( WICED_ETHERNET_INTERFACE, &ip_address, PING_TIMEOUT_MS, &elapsed_ms );
            if ( status == WICED_SUCCESS )
            {
                printf("Ping Reply : %lu ms\n", (unsigned long)elapsed_ms );
            }
            else if ( status == WICED_TIMEOUT )
            {
                printf("Ping timeout\n");
            }
            else
            {
                printf("Ping error\n");
            }
            wiced_rtos_delay_milliseconds(2000);
        }
        printf("PANU DEMO WICED: network down, exit.\n");
    }
}
#endif

/*
 * @section Packet Handler
 * 
 * @text The packet handler responds to various HCI Events.
 */

/* LISTING_START(packetHandler): Packet Handler */
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
/* LISTING_PAUSE */
    UNUSED(channel);

    uint8_t   event;
    bd_addr_t event_addr;
    bd_addr_t local_addr;
    uint16_t  uuid_source;
    uint16_t  uuid_dest;
    uint16_t  mtu;    
  
    /* LISTING_RESUME */
    switch (packet_type) {
		case HCI_EVENT_PACKET:
            event = hci_event_packet_get_type(packet);
            switch (event) {            
                /* @text When BTSTACK_EVENT_STATE with state HCI_STATE_WORKING
                 * is received and the example is started in client mode, the remote SDP BNEP query is started.
                 */
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                        if (!pairing_mode){
                            if (memcmp(remote_addr, invalid_addr, 6) == 0){
                                printf("No Bluetooth address stored for reconnect.\n");
                            } else {
                                schedule_outgoing_connection();
                            }
                        }
                    }
                    break;

                /* LISTING_PAUSE */
                case HCI_EVENT_PIN_CODE_REQUEST:
					// inform about pin code request
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
					break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // inform about user confirmation request
                    printf("SSP User Confirmation Auto accept\n");

                    if (pairing_mode){
                        hci_event_user_confirmation_request_get_bd_addr(packet, remote_addr);
#ifdef WICED_VERSION
                        // stored address in DCT after Link Key DB + LE Device DB
                        uint32_t offset = btstack_link_key_db_wiced_dct_get_storage_size() + le_device_db_wiced_dct_get_storage_size();
                        wiced_dct_write((void*)remote_addr, DCT_APP_SECTION, offset, 6);
                        printf("Stored remote address %s in DCT\n", bd_addr_to_str(remote_addr));
#endif
                        schedule_outgoing_connection();
                    }
                    break;

                /* LISTING_RESUME */

                /* @text BNEP_EVENT_CHANNEL_OPENED is received after a BNEP connection was established or 
                 * or when the connection fails. The status field returns the error code.
                 * 
                 * The TAP network interface is then configured. A data source is set up and registered with the 
                 * run loop to receive Ethernet packets from the TAP interface.
                 *
                 * The event contains both the source and destination UUIDs, as well as the MTU for this connection and
                 * the BNEP Channel ID, which is used for sending Ethernet packets over BNEP.
                 */  
				case BNEP_EVENT_CHANNEL_OPENED:
                    if (bnep_event_channel_opened_get_status(packet)) {
                        printf("BNEP channel open failed, status %02x\n", bnep_event_channel_opened_get_status(packet));
#ifdef AUTO_RECONNECT
                        schedule_outgoing_connection();
#endif
                    } else {
                        bnep_cid    = bnep_event_channel_opened_get_bnep_cid(packet);
                        uuid_source = bnep_event_channel_opened_get_source_uuid(packet);
                        uuid_dest   = bnep_event_channel_opened_get_destination_uuid(packet);
                        mtu         = bnep_event_channel_opened_get_mtu(packet);
                        memcpy(&event_addr, &packet[11], sizeof(bd_addr_t));
                        printf("BNEP connection open succeeded to %s source UUID 0x%04x dest UUID: 0x%04x, max frame size %u\n", bd_addr_to_str(event_addr), uuid_source, uuid_dest, mtu);
                        /* Setup network interface */
                        gap_local_bd_addr(local_addr);
                        btstack_network_up(local_addr);
                        printf("Network Interface '%s' activated\n", btstack_network_get_name());
#ifdef WICED_VERSION
                        network_up = 1;
                        wiced_rtos_set_semaphore(&panu_demo_wiced_semaphore);
#endif                        
                    }
					break;
                
                /* @text If there is a timeout during the connection setup, BNEP_EVENT_CHANNEL_TIMEOUT will be received
                 * and the BNEP connection  will be closed
                 */     
                case BNEP_EVENT_CHANNEL_TIMEOUT:
                    printf("BNEP channel timeout! Channel will be closed\n");
                    break;

                /* @text BNEP_EVENT_CHANNEL_CLOSED is received when the connection gets closed.
                 */
                case BNEP_EVENT_CHANNEL_CLOSED:
                    printf("BNEP channel closed\n");
#ifdef WICED_VERSION
                    printf("Signaling network demo to stop\n");
                    network_up = 0;
#endif                        
                    btstack_network_down();

#ifdef AUTO_RECONNECT
                    schedule_outgoing_connection();
#endif
                    break;

                /* @text BNEP_EVENT_CAN_SEND_NOW indicates that a new packet can be send. This triggers the send of a 
                 * stored network packet. The tap datas source can be enabled again
                 */
                case BNEP_EVENT_CAN_SEND_NOW:
                    if (network_buffer_len > 0) {
                        bnep_send(bnep_cid, (uint8_t*) network_buffer, network_buffer_len);
                        network_buffer_len = 0;
                        btstack_network_packet_sent();
                    }
                    break;
                    
                default:
                    break;
            }
            break;

        /* @text Ethernet packets from the remote device are received in the packet handler with type BNEP_DATA_PACKET.
         * It is forwarded to the TAP interface.
         */
        case BNEP_DATA_PACKET:
            // Write out the ethernet frame to the network interface
            btstack_network_process_packet(packet, size);
            break;            
            
        default:
            break;
    }
}
/* LISTING_END */

/*
 * @section Network packet handler
 * 
 * @text A pointer to the network packet is stored and a BNEP_EVENT_CAN_SEND_NOW requested
 */

/* LISTING_START(networkPacketHandler): Network Packet Handler */
static void network_send_packet_callback(const uint8_t * packet, uint16_t size){
    network_buffer = packet;
    network_buffer_len = size;
    bnep_request_can_send_now_event(bnep_cid);
}
/* LISTING_END */

int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){

    (void)argc;
    (void)argv;

    memcpy(remote_addr, invalid_addr, 6);

#ifdef WICED_VERSION
    // detect Button 1 hold on boot
    pairing_mode = wiced_gpio_input_get( WICED_BUTTON2 ) ? 0 : 1;  /* The button has inverse logic */
    printf("Button 2 pressed/Pairing Mode: %u\n", pairing_mode);

    // if not in pairing mode, fetch stored address
    if (!pairing_mode){
        // stored address in DCT after Link Key DB + LE Device DB
        uint32_t offset = btstack_link_key_db_wiced_dct_get_storage_size() + le_device_db_wiced_dct_get_storage_size();
        void * entry = NULL;
        wiced_dct_read_lock((void*) &entry, WICED_FALSE, DCT_APP_SECTION, offset, 6);
        memcpy(remote_addr, entry, 6);
        // read unlock
        wiced_dct_read_unlock((void*) entry, WICED_FALSE);
        printf("Stored remote address %s\n", bd_addr_to_str(remote_addr));
    }

    // start network thread
    network_up = 0;
    wiced_rtos_init_semaphore(&panu_demo_wiced_semaphore);
    wiced_rtos_create_thread(&panu_demo_wiced_thread, WICED_APPLICATION_PRIORITY, "panu-demo", &panu_demo_wiced, 1024, NULL);
#endif

    panu_setup();

    // Turn on the device 
    hci_power_control(HCI_POWER_ON);
    return 0;
}

/* EXAMPLE_END */
/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-  */

