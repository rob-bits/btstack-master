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

/*
 *  btstack_uart.h
 *
 *  Common types for UART transports
 *
 */

#ifndef __BTSTACK_UART_H
#define __BTSTACK_UART_H

#include <stdint.h>

typedef struct {
    uint32_t   baudrate;
    int        flowcontrol;
    const char *device_name;
} btstack_uart_config_t;

typedef enum {
    // UART active, sleep off
    BTSTACK_UART_SLEEP_OFF = 0,
    // used for eHCILL
    BTSTACK_UART_SLEEP_RTS_HIGH_WAKE_ON_CTS_PULSE,
    // used for H5 and for eHCILL without support for wake on CTS pulse
    BTSTACK_UART_SLEEP_RTS_LOW_WAKE_ON_RX_EDGE, 

} btstack_uart_sleep_mode_t;

typedef enum { 
    BTSTACK_UART_SLEEP_MASK_RTS_HIGH_WAKE_ON_CTS_PULSE  = 1 << BTSTACK_UART_SLEEP_RTS_HIGH_WAKE_ON_CTS_PULSE,
    BTSTACK_UART_SLEEP_MASK_RTS_LOW_WAKE_ON_RX_EDGE     = 1 << BTSTACK_UART_SLEEP_RTS_LOW_WAKE_ON_RX_EDGE
} btstack_uart_sleep_mode_mask_t;

#endif
