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

#include "hal_audio.h"
#include "stm32f4_discovery_audio.h"

#define OUTPUT_BUFFER_NUM_SAMPLES       512
#define NUM_OUTPUT_BUFFERS              2

static void (*audio_played_handler)(void);
static int started;
static volatile int playing_buffer;
static int buffer_to_fill;

// our storage
static uint16_t output_buffer[NUM_OUTPUT_BUFFERS * OUTPUT_BUFFER_NUM_SAMPLES * 2];   // stereo

void  BSP_AUDIO_OUT_HalfTransfer_CallBack(void){
	playing_buffer = 1;
	(*audio_played_handler)();
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void){
	playing_buffer = 0;
	(*audio_played_handler)();
}

/**
 * @brief Setup audio codec for specified samplerate
 * @param Channels
 * @param Sample rate
 */
void hal_audio_init(uint8_t channels, uint32_t sample_rate){
	BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_BOTH, 100, sample_rate);
	buffer_to_fill = 0;
	playing_buffer = 0;

	// TODO: configure for circular DMA
}

/**
 * @brief Set callback to call when audio was sent
 * @param handler
 */
void hal_audio_set_audio_played(void (*handler)(void)){
	audio_played_handler = handler;
}

/**
 * @brief Set callback to call when audio was recorded
 * @param handler
 */
void hal_audio_set_audio_recorded(void (*handler)(const uint16_t * samples, uint16_t num_samples)){
	UNUSED(handler);
}

/**
 * @brief Get number of output buffers in HAL
 * @returns num buffers
 */
uint16_t hal_audio_get_num_output_buffers(void){
	return NUM_OUTPUT_BUFFERS;
}

/**
 * @brief Get size of single output buffer in HAL
 * @returns buffer size
 */
uint16_t hal_audio_get_num_output_buffer_samples(void){
	return OUTPUT_BUFFER_NUM_SAMPLES;
}

/**
 * @brief Reserve output buffer
 * @returns buffer
 */
uint16_t * hal_audio_reserve_output_buffer(void){
	if (started && buffer_to_fill == playing_buffer) return NULL;

	int16_t * buffer = buffer_to_fill == 0 ? output_buffer : &output_buffer[OUTPUT_BUFFER_NUM_SAMPLES * 2];
	buffer_to_fill = (buffer_to_fill + 1) & 1;
	return buffer;
}

/**
 * @brief Start stream
 */
void hal_audio_start(void){
	started = 1;
	playing_buffer = 0;
	BSP_AUDIO_OUT_Play(output_buffer, OUTPUT_BUFFER_NUM_SAMPLES);
}

/**
 * @brief Close audio codec
 */
void hal_audio_close(void){
	started = 0;
	BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
}
