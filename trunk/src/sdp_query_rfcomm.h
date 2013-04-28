/*
 * Copyright (C) 2009-2013 by Matthias Ringwald
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
 * Please inquire about commercial licensing options at btstack@ringwald.ch
 *
 */

/*
 *  sdp_rfcomm_query.h
 */

#include <btstack/utils.h>
#include "sdp_parser.h"

// Basic SDP Query event type
typedef struct sdp_query_event {
    uint8_t type;
} sdp_query_event_t;

// SDP Query event to indicate that query is complete.
typedef struct sdp_query_complete_event {
    uint8_t type;
    uint8_t status; // 0 == OK
} sdp_query_complete_event_t;

// SDP Query RFCOMM event to deliver channel number and service name byte by byte.
typedef struct sdp_query_rfcomm_service_event {
    uint8_t type;
    uint8_t channel_nr;
    uint8_t * service_name;
} sdp_query_rfcomm_service_event_t;


void sdp_query_rfcomm_init(void);

// Searches SDP records on a remote device for RFCOMM services with the given UUID.
void sdp_query_rfcomm_channel_and_name_for_service_with_uuid(bd_addr_t remote, uint16_t uuid);

// Searches SDP records on a remote device for RFCOMM services with a given service search pattern.
void sdp_query_rfcomm_channel_and_name_for_service_with_service_search_pattern(bd_addr_t remote, uint8_t * des_serviceSearchPattern);

// Registers a callback to receive RFCOMM service and done event. 
void sdp_query_rfcomm_register_callback(void(*sdp_app_callback)(sdp_query_event_t * event, void * context), void * context);
