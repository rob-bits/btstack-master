/*
 * Copyright (C) 2018 BlueKitchen GmbH
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

#ifndef __HAL_AUDIO_H
#define __HAL_AUDIO_H

#include <stdint.h>

/*
 *  hal_audio.h
 *
 *  Hardware abstraction layer that provides circular audio playback and recording
 *  Assumptions: 
 *  - num buffers >= 2
 *  - after start, buffers are played back in sequence 0, 1, .., n
 *  - after a buffer is played back, the callback is called with tbe buffer index
 */

/**
 * @brief Setup audio codec for specified samplerate
 * @param Channels
 * @param Sample rate
 */
void hal_audio_init(uint8_t channels, uint32_t sample_rate);

/**
 * @brief Set callback to call when audio was sent
 * @param handler
 */
void hal_audio_set_audio_played(void (*handler)(uint8_t buffer_index));

/**
 * @brief Set callback to call when audio was recorded
 * @param handler
 */
void hal_audio_set_audio_recorded(void (*handler)(const uint16_t * samples, uint16_t num_samples));

/**
 * @brief Get number of output buffers in HAL
 * @returns num buffers
 */
uint16_t hal_audio_get_num_output_buffers(void);

/**
 * @brief Get size of single output buffer in HAL
 * @returns buffer size
 */
uint16_t hal_audio_get_num_output_buffer_samples(void);

/**
 * @brief Gey output buffer
 * @param buffer index
 * @returns buffer
 */
uint16_t * hal_audio_get_output_buffer(uint8_t buffer_index);

/**
 * @brief Start stream
 */
void hal_audio_start(void);

/**
 * @brief Close audio codec
 */
void hal_audio_close(void);

#endif
