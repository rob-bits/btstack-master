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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "btstack_debug.h"
#include "btstack_audio.h"

#ifdef HAVE_PORTAUDIO

#define PA_SAMPLE_TYPE      paInt16

#include "btstack_ring_buffer.h"
#include <portaudio.h>

static PaStream * stream;
static int media_initialized = 0;
static void (*playback_callback)(uint16_t * buffer, uint16_t num_samples);

static int portaudioCallback( const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo* timeInfo,
                           PaStreamCallbackFlags statusFlags,
                           void *userData ) {

    /** portaudioCallback is called from different thread, don't use hci_dump / log_info here without additional checks */

    (void) timeInfo; /* Prevent unused variable warnings. */
    (void) statusFlags;
    (void) inputBuffer;
    (void) userData;
    (void) outputBuffer;
    (void) framesPerBuffer;
    
    if (playback_callback){
        (*playback_callback)(outputBuffer, framesPerBuffer);
    } else {
        memset(outputBuffer, 0, 2*2*framesPerBuffer);
    }

    // just redirect to our callbacl
    return 0;
}

static int btstack_audio_portaudio_init(uint8_t channels, uint32_t samplerate){
    if (media_initialized) return 0;

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
           0,
           paClipOff,           /* we won't output out of range samples so don't bother clipping them */
           portaudioCallback,      /* use callback */
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
    /* -- start stream -- */
    PaError err = Pa_StartStream(stream);
    if (err != paNoError){
        log_error("Error starting the stream: \"%s\"\n",  Pa_GetErrorText(err));
        return;
    }
    playback_callback = callback;
}

static void btstack_audio_portaudio_close(void){
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
