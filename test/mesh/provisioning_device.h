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
 *  provisioning_device.h
 */

#ifndef __PROVISIONING_DEVICE_H
#define __PROVISIONING_DEVICE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Init Provisioning in Device Role with device UUID
 * @param device_uuid
 */
void provisioning_device_init(const uint8_t * device_uuid);

/**
 * @brief Public Key OOB Available
 * @note Requires ability to transmit public key out-of-band
 */
void provisioning_device_set_public_key_oob_available(void);

/**
 * @brief Static OOB Available
 * @param static_oob_len in bytes
 * @param static_oob_data
 */
void provisioning_device_set_static_oob(uint16_t static_oob_len, const uint8_t * static_oob_data);

/**
 * @brief Configure Output OOB Options
 * @param supported_output_oob_action_types bitfield
 * @param max_oob_output_size in bytes
 */
void provisioning_device_set_output_oob_actions(uint16_t supported_output_oob_action_types, uint8_t max_oob_output_size);

/**
 * @brief Configure Input OOB Options
 * @param supported_input_oob_action_types bitfield
 * @param max_oob_input_size in bytes
 */
void provisioning_device_set_input_oob_actions(uint16_t supported_input_oob_action_types, uint8_t max_oob_input_size);

#ifdef __cplusplus
} /* end of extern "C" */
#endif

#endif
