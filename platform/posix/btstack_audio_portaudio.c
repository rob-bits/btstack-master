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


#include <stdint.h>
#include <string.h>
#include "btstack_debug.h"
#include "btstack_audio.h"
#include "btstack_run_loop.h"

#ifdef HAVE_PORTAUDIO

#define PA_SAMPLE_TYPE               paInt16
#define NUM_FRAMES_PER_PA_BUFFER       512
#define NUM_BUFFERS                      3
#define DRIVER_POLL_INTERVAL_MS          5

#include <portaudio.h>

// config
static int                    num_channels;
static int                    num_bytes_per_sample;

// portaudio
static PaStream * stream;

// client
static void (*playback_callback)(uint16_t * buffer, uint16_t num_samples);

// output buffer
static uint16_t               output_buffer_a[NUM_FRAMES_PER_PA_BUFFER * 2];   // stereo
static uint16_t               output_buffer_b[NUM_FRAMES_PER_PA_BUFFER * 2];   // stereo
static uint16_t               output_buffer_c[NUM_FRAMES_PER_PA_BUFFER * 2];   // stereo
static uint16_t             * output_buffers[NUM_BUFFERS] = { output_buffer_a, output_buffer_b, output_buffer_c};
static int                    output_buffer_to_play;
static int                    output_buffer_to_fill;

// timer to fill output ring buffer
static btstack_timer_source_t  driver_timer;

static int portaudio_callback( const void *                     inputBuffer, 
                               void *                           outputBuffer,
                               unsigned long                    samples_per_buffer,
                               const PaStreamCallbackTimeInfo * timeInfo,
                               PaStreamCallbackFlags            statusFlags,
                               void *                           userData ) {

    /** portaudio_callback is called from different thread, don't use hci_dump / log_info here without additional checks */

    (void) timeInfo; /* Prevent unused variable warnings. */
    (void) statusFlags;
    (void) inputBuffer;
    (void) userData;
    (void) samples_per_buffer;

    // printf("PLAY: play #%u, fill #%u\n", output_buffer_to_play, output_buffer_to_fill);
    
    // fill from our ringbuffer
    memcpy(outputBuffer, output_buffers[output_buffer_to_play], NUM_FRAMES_PER_PA_BUFFER * num_bytes_per_sample);

    // next
    output_buffer_to_play = (output_buffer_to_play + 1 ) % NUM_BUFFERS;

    return 0;
}

static void driver_timer_handler(btstack_timer_source_t * ts){

    // printf("FILL: play #%u, fill #%u\n", output_buffer_to_play, output_buffer_to_fill);

    // buffer ready
    if (output_buffer_to_play != output_buffer_to_fill){
        (*playback_callback)(output_buffers[output_buffer_to_fill], NUM_FRAMES_PER_PA_BUFFER);

        // next
        output_buffer_to_fill = (output_buffer_to_fill + 1 ) % NUM_BUFFERS;
    }

    // re-set timer
    btstack_run_loop_set_timer(ts, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(ts);
}

static int btstack_audio_portaudio_init(uint8_t channels, uint32_t samplerate){

    num_channels = channels;
    num_bytes_per_sample = 2 * channels;

    // int frames_per_buffer = configuration.frames_per_buffer;
    PaError err;
    PaStreamParameters outputParameters;
    const PaDeviceInfo *deviceInfo;

    /* -- initialize PortAudio -- */
    err = Pa_Initialize();
    if (err != paNoError){
        log_error("Error initializing portaudio: \"%s\"\n",  Pa_GetErrorText(err));
        return err;
    } 
    /* -- setup input and output -- */
    outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
    outputParameters.channelCount = channels;
    outputParameters.sampleFormat = PA_SAMPLE_TYPE;
    outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultHighOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;
    deviceInfo = Pa_GetDeviceInfo( outputParameters.device );
    log_info("PortAudio: Output device: %s", deviceInfo->name);
    /* -- setup stream -- */
    err = Pa_OpenStream(
           &stream,
           NULL,                /* &inputParameters */
           &outputParameters,
           samplerate,
           NUM_FRAMES_PER_PA_BUFFER,
           paClipOff,           /* we won't output out of range samples so don't bother clipping them */
           portaudio_callback,  /* use callback */
           NULL );   
    
    if (err != paNoError){
        log_error("Error initializing portaudio: \"%s\"\n",  Pa_GetErrorText(err));
        return err;
    }
    log_info("PortAudio: stream opened");

    const PaStreamInfo * stream_info = Pa_GetStreamInfo(stream);
    log_info("PortAudio: Input  latency: %f", stream_info->inputLatency);
    log_info("PortAudio: Output latency: %f", stream_info->outputLatency);

    return 0;
}

static void btstack_audio_portaudio_set_playback_callback(void (*callback)(uint16_t * buffer, uint16_t num_samples)){

    // fill buffer once
    (*callback)(output_buffer_a, NUM_FRAMES_PER_PA_BUFFER);
    (*callback)(output_buffer_b, NUM_FRAMES_PER_PA_BUFFER);
    output_buffer_to_play = 0;
    output_buffer_to_fill = 2;

    /* -- start stream -- */
    PaError err = Pa_StartStream(stream);
    if (err != paNoError){
        log_error("Error starting the stream: \"%s\"\n",  Pa_GetErrorText(err));
        return;
    }
    playback_callback = callback;

    // start timer
    btstack_run_loop_set_timer_handler(&driver_timer, &driver_timer_handler);
    btstack_run_loop_set_timer(&driver_timer, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(&driver_timer);
}

static void btstack_audio_portaudio_close(void){

    // stop timer
    btstack_run_loop_remove_timer(&driver_timer);

    log_info("PortAudio: Stream closed");
    PaError err = Pa_StopStream(stream);
    if (err != paNoError){
        log_error("Error stopping the stream: \"%s\"",  Pa_GetErrorText(err));
        return;
    } 
    err = Pa_CloseStream(stream);
    if (err != paNoError){
        log_error("Error closing the stream: \"%s\"",  Pa_GetErrorText(err));
        return;
    } 
    err = Pa_Terminate();
    if (err != paNoError){
        log_error("Error terminating portaudio: \"%s\"",  Pa_GetErrorText(err));
        return;
    } 
}

static const btstack_audio_t btstack_audio_portaudio = {
    /* int (*init)(uint8_t channels, uint32_t baudrate);*/      &btstack_audio_portaudio_init,
    /* void (*set_playback_callback)(void (*callback)(...));*/  &btstack_audio_portaudio_set_playback_callback,
    /* void (*close)(void); */                                  &btstack_audio_portaudio_close
};

const btstack_audio_t * btstack_audio_portaudio_get_instance(void){
    return &btstack_audio_portaudio;
}

#endif
