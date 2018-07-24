/*
 * Copyright (C) 2016 BlueKitchen GmbH
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

#define __BTSTACK_FILE__ "a2dp_source_demo.c"
#define API_DEBUG
/*
 * a2dp_source_demo.c
 */

// *****************************************************************************
/* EXAMPLE_START(a2dp_source_demo): Serve audio stream and handle remote playback control and queries.
 *
 * @text This  A2DP Source example demonstrates how to send an audio data stream 
 * to a remote A2DP Sink device and how to switch between two audio data sources.  
 * In addition, the AVRCP Target is used to answer queries on currently played media,
 * as well as to handle remote playback control, i.e. play, stop, repeat, etc.
 *
 * @test To test with a remote device, e.g. a Bluetooth speaker,
 * set the device_addr_string to the Bluetooth address of your 
 * remote device in the code, and use the UI to connect and start playback. 
 * Tap SPACE on the console to show the available commands.
 */
// *****************************************************************************


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btstack.h"
#include "hxcmod.h"
#include "mods/mod.h"

#define USE_SD_CARD_AUDIO
#ifdef USE_SD_CARD_AUDIO
#include <unistd.h>
#include <fcntl.h>
#include <sys/unistd.h>
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#endif //USE_SD_CARD_AUDIO
#include "driver/gpio.h"
#define GPIO_OUTPUT_IO_0    32
#define GPIO_OUTPUT_IO_1    33
#define GPIO_OUTPUT_IO_2    17
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1)| (1ULL<<GPIO_OUTPUT_IO_2))
// logarithmic volume reduction, samples are divided by 2^x
// #define VOLUME_REDUCTION 3
// #undef  HAVE_BTSTACK_STDIN

#define AVRCP_BROWSING_ENABLED 0

#define NUM_CHANNELS                2
#define A2DP_SAMPLE_RATE            44100
#define A2DP_SAMPLE_RATE_SHIFT_DIV    10 //1024
#define A2DP_SAMPLE_RATE_CALC    (45148)
//#define A2DP_SAMPLE_RATE_CALC    ((A2DP_SAMPLE_RATE / 1000) << A2DP_SAMPLE_RATE_SHIFT_DIV)
#define A2DP_SAMPLE_RATE_2            44100
#define BYTES_PER_AUDIO_SAMPLE      (2*NUM_CHANNELS)
#define AUDIO_TIMEOUT_MS            5
#define TABLE_SIZE_441HZ            100

#define SBC_STORAGE_SIZE 1030

#define BT_NUM_OF_DEVICES 2
#define BD_ADDR_LEN 6

typedef enum {
    STREAM_SINE = 0,
    STREAM_MOD,
    STREAM_PTS_TEST,
    STREAM_FROM_SD
} stream_data_source_t;
    
typedef struct {
    uint16_t a2dp_cid;
    uint8_t  local_seid;
    uint8_t  stream_opened;
    uint16_t avrcp_cid;

    uint32_t time_audio_data_sent; // ms
    uint32_t acc_num_missed_samples;
    uint32_t samples_ready;
    btstack_timer_source_t audio_timer;
    uint8_t  streaming;
    int      max_media_payload_size;
    
    uint8_t  sbc_storage[SBC_STORAGE_SIZE];
    uint16_t sbc_storage_count;
    uint8_t  sbc_ready_to_send;
    uint16_t timer_started;
    int last_l_sample;
    int last_r_sample;
} a2dp_media_sending_context_t;

typedef struct {
    uint8_t config;
    uint8_t stream;
    uint8_t connected_devices;
    uint8_t alter;
    uint8_t stream_both;
    uint8_t debug;
}my_a2dp_idx_context_t;

typedef struct {
    uint8_t flag_0;
    uint8_t flag_1;
    uint8_t flag_2;
}gpio_flags_context_t;

static  uint8_t media_sbc_codec_capabilities[] = {
    (AVDTP_SBC_44100 << 4) | AVDTP_SBC_STEREO,
    0xFF,//(AVDTP_SBC_BLOCK_LENGTH_16 << 4) | (AVDTP_SBC_SUBBANDS_8 << 2) | AVDTP_SBC_ALLOCATION_METHOD_LOUDNESS,
    2, 53
}; 

static const int16_t sine_int16[] = {
     0,    2057,    4107,    6140,    8149,   10126,   12062,   13952,   15786,   17557,
 19260,   20886,   22431,   23886,   25247,   26509,   27666,   28714,   29648,   30466,
 31163,   31738,   32187,   32509,   32702,   32767,   32702,   32509,   32187,   31738,
 31163,   30466,   29648,   28714,   27666,   26509,   25247,   23886,   22431,   20886,
 19260,   17557,   15786,   13952,   12062,   10126,    8149,    6140,    4107,    2057,
     0,   -2057,   -4107,   -6140,   -8149,  -10126,  -12062,  -13952,  -15786,  -17557,
-19260,  -20886,  -22431,  -23886,  -25247,  -26509,  -27666,  -28714,  -29648,  -30466,
-31163,  -31738,  -32187,  -32509,  -32702,  -32767,  -32702,  -32509,  -32187,  -31738,
-31163,  -30466,  -29648,  -28714,  -27666,  -26509,  -25247,  -23886,  -22431,  -20886,
-19260,  -17557,  -15786,  -13952,  -12062,  -10126,   -8149,   -6140,   -4107,   -2057,
};

typedef struct {
    int reconfigure;
    int num_channels;
    int sampling_frequency;
    int channel_mode;
    int block_length;
    int subbands;
    int allocation_method;
    int min_bitpool_value;
    int max_bitpool_value;
    int frames_per_buffer;
} avdtp_media_codec_configuration_sbc_t;

static btstack_packet_callback_registration_t hci_event_callback_registration;

// pts:             static const char * device_addr_string = "00:1B:DC:08:0A:A5";
// mac 2013:        static const char * device_addr_string = "84:38:35:65:d1:15";
// phone 2013:      static const char * device_addr_string = "D8:BB:2C:DF:F0:F2";
// Minijambox:      
static const char * device_addr_string_0 = "fc:58:fa:49:20:f1";  //PPA11BT
static const char * device_addr_string_1 = "00:58:02:cc:02:64";  //BTS-06
// Philips SHB9100: static const char * device_addr_string = "00:22:37:05:FD:E8";
// RT-B6:           static const char * device_addr_string = "00:75:58:FF:C9:7D";
// BT dongle:       static const char * device_addr_string = "00:1A:7D:DA:71:0A";
// Sony MDR-ZX330BT static const char * device_addr_string = "00:18:09:28:50:18";
// Panda (BM6)      static const char * device_addr_string = "4F:3F:66:52:8B:E0";

//static bd_addr_t device_addr;
static uint8_t device_addr[BT_NUM_OF_DEVICES][BD_ADDR_LEN];
static uint8_t sdp_a2dp_source_service_buffer[150];
static uint8_t sdp_avrcp_target_service_buffer[200];
static avdtp_media_codec_configuration_sbc_t sbc_configuration;
static btstack_sbc_encoder_state_t sbc_encoder_state;

static uint8_t media_sbc_codec_configuration[4];
static a2dp_media_sending_context_t media_tracker[BT_NUM_OF_DEVICES] = {0};
static my_a2dp_idx_context_t idx_handler = {0};
static gpio_flags_context_t gpio = {0};
static avdtp_stream_endpoint_t * local_stream_endpoint;
int16_t pcm_frame[256*NUM_CHANNELS] = {0}; //< put outside of this scope and make it static for faster process?
uint16_t l2cap_media_cid[BT_NUM_OF_DEVICES] = {0x42, 0x42};
static uint8_t deviceIdx = 0; //variable for storing the current connecting device ID
volatile int m_sample = 0; //variable of the sd card output

static stream_data_source_t data_source;

static int sine_phase = 0;

static int hxcmod_initialized;
static modcontext mod_context;
static tracker_buffer_state trkbuf;


/* AVRCP Target context START */
static const uint8_t subunit_info[] = {
    0,0,0,0,
    1,1,1,1,
    2,2,2,2,
    3,3,3,3,
    4,4,4,4,
    5,5,5,5,
    6,6,6,6,
    7,7,7,7
};

static uint32_t company_id = 0x112233;
static uint8_t companies_num = 1;
static uint8_t companies[] = {
    0x00, 0x19, 0x58 //BT SIG registered CompanyID
};

static uint8_t events_num = 13;
static uint8_t events[] = {
    AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED,
    AVRCP_NOTIFICATION_EVENT_TRACK_CHANGED,
    AVRCP_NOTIFICATION_EVENT_TRACK_REACHED_END,
    AVRCP_NOTIFICATION_EVENT_TRACK_REACHED_START,
    AVRCP_NOTIFICATION_EVENT_PLAYBACK_POS_CHANGED,
    AVRCP_NOTIFICATION_EVENT_BATT_STATUS_CHANGED,
    AVRCP_NOTIFICATION_EVENT_SYSTEM_STATUS_CHANGED,
    AVRCP_NOTIFICATION_EVENT_PLAYER_APPLICATION_SETTING_CHANGED,
    AVRCP_NOTIFICATION_EVENT_NOW_PLAYING_CONTENT_CHANGED,
    AVRCP_NOTIFICATION_EVENT_AVAILABLE_PLAYERS_CHANGED,
    AVRCP_NOTIFICATION_EVENT_ADDRESSED_PLAYER_CHANGED,
    AVRCP_NOTIFICATION_EVENT_UIDS_CHANGED,
    AVRCP_NOTIFICATION_EVENT_VOLUME_CHANGED
};

typedef struct {
    uint8_t track_id[8];
    uint32_t song_length_ms;
    avrcp_playback_status_t status;
    uint32_t song_position_ms; // 0xFFFFFFFF if not supported
} avrcp_play_status_info_t;

avrcp_track_t tracks[] = {
    {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}, 1, "Sine", "Generated", "AVRCP Demo", "monotone", 12345},
    {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}, 2, "Nao-deceased", "Decease", "AVRCP Demo", "vivid", 12345},
    {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03}, 3, "aaaaaa", "Decease", "AVRCP Demo", "vivid", 12345},
    {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04}, 4, "Play relax", "Decease", "AVRCP Demo", "vivid", 12345},
};
int current_track_index;
avrcp_play_status_info_t play_info;

/* AVRCP Target context END */

/* @section Main Application Setup
 *
 * @text The Listing MainConfiguration shows how to setup AD2P Source and AVRCP Target services. 
 */

/* LISTING_START(MainConfiguration): Setup Audio Source and AVRCP Target services */
static void a2dp_source_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * event, uint16_t event_size);
static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
#ifdef HAVE_BTSTACK_STDIN
static void stdin_process(char cmd);
#endif

static int a2dp_source_and_avrcp_services_init(void){
    // Register for HCI events.

#ifdef API_DEBUG
    printf("API - in the function %s \n", __func__);
#endif
    hci_event_callback_registration.callback = &a2dp_source_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();
#ifdef API_DEBUG
    printf("API - l2cap_init() finished \n");
#endif
    // Initialize  A2DP Source.
    a2dp_source_init();
    a2dp_source_register_packet_handler(&a2dp_source_packet_handler);

#ifdef API_DEBUG
    printf("API - a2dp_source_init() finished \n");
#endif

    // Create stream endpoint for device 0.
    local_stream_endpoint = a2dp_source_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_SBC, media_sbc_codec_capabilities, sizeof(media_sbc_codec_capabilities), media_sbc_codec_configuration, sizeof(media_sbc_codec_configuration));
    if (!local_stream_endpoint){
        printf("A2DP Source: not enough memory to create local stream endpoint for dev0\n");
        return 1;
    }
    media_tracker[0].local_seid = avdtp_local_seid(local_stream_endpoint);

#ifdef API_DEBUG
    printf("API - sep 0 created \n");
#endif
    // Create stream endpoint for device 1.
    local_stream_endpoint = a2dp_source_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_SBC, media_sbc_codec_capabilities, sizeof(media_sbc_codec_capabilities), media_sbc_codec_configuration, sizeof(media_sbc_codec_configuration));
    if (!local_stream_endpoint){
        printf("A2DP Source: not enough memory to create local stream endpoint for dev1\n");
        return 1;
    }
    media_tracker[1].local_seid = avdtp_local_seid(local_stream_endpoint);
    //media_tracker[1].local_seid = media_tracker[0].local_seid;

#ifdef API_DEBUG
    printf("API - sep 1 created \n");
#endif

    // Initialize AVRCP Target.
    avrcp_target_init();
    avrcp_target_register_packet_handler(&avrcp_target_packet_handler);
    
    // Initialize SDP, 
    sdp_init();
    
    // Create  A2DP Source service record and register it with SDP.
    memset(sdp_a2dp_source_service_buffer, 0, sizeof(sdp_a2dp_source_service_buffer));
    a2dp_source_create_sdp_record(sdp_a2dp_source_service_buffer, 0x10002, 1, NULL, NULL);
    sdp_register_service(sdp_a2dp_source_service_buffer);
    
    // Create AVRCP target service record and register it with SDP.
    memset(sdp_avrcp_target_service_buffer, 0, sizeof(sdp_avrcp_target_service_buffer));
    avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer, 0x10001, AVRCP_BROWSING_ENABLED, 1, NULL, NULL);
    sdp_register_service(sdp_avrcp_target_service_buffer);

    // Set local name with a template Bluetooth address, that will be automatically
    // replaced with a actual address once it is available, i.e. when BTstack boots
    // up and starts talking to a Bluetooth module.
    gap_set_local_name("A2DP Source 00:00:00:00:00:00");
    gap_discoverable_control(0);
    gap_set_class_of_device(0x200408);
    
    hxcmod_initialized = hxcmod_init(&mod_context);
    if (hxcmod_initialized){
        hxcmod_setcfg(&mod_context, A2DP_SAMPLE_RATE, 16, 1, 1, 1);
        hxcmod_load(&mod_context, (void *) &mod_data, mod_len);
        printf("loaded mod '%s', size %u\n", mod_name, mod_len);
    }
    
    // Parse human readable Bluetooth address.
    sscanf_bd_addr(device_addr_string_0, &device_addr[0][0]);
    sscanf_bd_addr(device_addr_string_1, &device_addr[1][0]);

#ifdef API_DEBUG
    printf("API - init finished! \n");
#endif

#ifdef HAVE_BTSTACK_STDIN
    btstack_stdin_setup(stdin_process);
#endif
    return 0;
}
/* LISTING_END */
static void a2dp_demo_send_media_packet(void){
    int num_bytes_in_frame = btstack_sbc_encoder_sbc_buffer_length();
    int bytes_in_storage;
    uint8_t i;

    gpio.flag_2 ^= 0x01; //toogle the GPIO flag
    gpio_set_level(GPIO_OUTPUT_IO_2,gpio.flag_2);

    bytes_in_storage = media_tracker[0].sbc_storage_count;
    uint8_t num_frames = bytes_in_storage / num_bytes_in_frame;

    // if(!idx_handler.stream_both)
    // {
        // deviceIdx = idx_handler.stream; //the sink device is selected by the idx_handler.stream switch if stream_both is not set
    // }
    a2dp_source_stream_send_media_payload(media_tracker[idx_handler.stream].a2dp_cid, media_tracker[idx_handler.stream].local_seid, media_tracker[0].sbc_storage, bytes_in_storage, num_frames, 0);

    if(!deviceIdx)
    {
        if(idx_handler.stream_both)
        {
            local_stream_endpoint = avdtp_stream_endpoint_for_seid(media_tracker[1].local_seid, a2dp_get_source_context());
            local_stream_endpoint->send_stream = 1;
            local_stream_endpoint->connection->wait_to_send_initiator = 1;
            l2cap_request_can_send_now_event(l2cap_media_cid[1]);
            deviceIdx = 1;
        }
        else
        {
            media_tracker[0].sbc_storage_count = 0;
            media_tracker[0].sbc_ready_to_send = 0;
        }
    }
    else
    {
        media_tracker[0].sbc_storage_count = 0;
        media_tracker[0].sbc_ready_to_send = 0;
        deviceIdx = 0;
    }
}

static void produce_sine_audio(int16_t * pcm_buffer, int num_samples_to_write){
    int count;
    sine_phase = media_tracker[idx_handler.stream].last_l_sample; //restore the last sample
    for (count = 0; count < num_samples_to_write ; count++){
        pcm_buffer[count * 2]     = sine_int16[sine_phase];
        pcm_buffer[count * 2 + 1] = sine_int16[sine_phase];
        sine_phase++;
        if (sine_phase >= TABLE_SIZE_441HZ){
            sine_phase -= TABLE_SIZE_441HZ;
        }
    }
    media_tracker[idx_handler.stream].last_l_sample = sine_phase; //store the last sample
}

#ifdef USE_SD_CARD_AUDIO
static void produce_sd_audio(uint8_t * pcm_buffer, int num_samples_to_write)
{
    int count;
    uint8_t temp_buffer;
    uint16_t i;

    if (num_samples_to_write < 0 || pcm_buffer == NULL)
    {
        return;
    }

    count = read(m_sample, (unsigned short *) &pcm_buffer[0], num_samples_to_write); //read from sd card len number of samples to the data buffer
    if (count < num_samples_to_write)
    {
        lseek(m_sample, 0, SEEK_SET);
    }

    printf("audio data: %d \n",num_samples_to_write);
    if(idx_handler.debug)
    {
        for(i = 0;i<num_samples_to_write;i++)
        {
            printf("%d",pcm_buffer[i]);
            idx_handler.debug = 0;
        }
    }
    printf("\n");
    //swap the bytes in the array
    // for(i=0;i<count;i+=2)
    // {
        // temp_buffer = pcm_buffer[i]; //store the ith sample
        // pcm_buffer[i] = pcm_buffer[i+1]; //over write the ith data with the i+1th data
        // pcm_buffer[i+1] = temp_buffer; //store the ith data
    // }
}
#endif //USE_SD_CARD_AUDIO

static void produce_mod_audio(int16_t * pcm_buffer, int num_samples_to_write){
    mod_context.last_l_sample = media_tracker[idx_handler.stream].last_l_sample; //restore the last samples
    mod_context.last_r_sample = media_tracker[idx_handler.stream].last_r_sample; //restore the last samples
    hxcmod_fillbuffer(&mod_context, (unsigned short *) &pcm_buffer[0], num_samples_to_write, &trkbuf);
    media_tracker[idx_handler.stream].last_l_sample = mod_context.last_l_sample; //store the last samples
    media_tracker[idx_handler.stream].last_r_sample = mod_context.last_r_sample; //store the last samples
}

static void produce_audio(int16_t * pcm_buffer, int num_samples){
    switch (data_source){
        case STREAM_FROM_SD:
#ifdef USE_SD_CARD_AUDIO
            produce_sd_audio((uint8_t *) pcm_buffer, 2*num_samples);
#endif //USE_SD_CARD_AUDIO
            break;
        case STREAM_SINE:
            produce_sine_audio(pcm_buffer, num_samples);
            break;
        case STREAM_MOD:
            produce_mod_audio(pcm_buffer, num_samples);
            break;
        default:
            break;
    }    
#ifdef VOLUME_REDUCTION
    int i;
    for (i=0;i<num_samples*2;i++){
        if (pcm_buffer[i] > 0){
            pcm_buffer[i] =     pcm_buffer[i]  >> VOLUME_REDUCTION;
        } else {
            pcm_buffer[i] = -((-pcm_buffer[i]) >> VOLUME_REDUCTION);
        }
    }
#endif
}

static int my_a2dp_fill_sbc_audio_buffer(uint8_t device){
    // perform sbc encodin
    int total_num_bytes_read = 0;
    uint8_t * sbc_frame;
    uint16_t sbc_frame_size;
    unsigned int num_audio_samples_per_sbc_buffer = btstack_sbc_encoder_num_audio_frames(); //context->s16NumOfSubBands * context->s16NumOfBlocks: sbc_configuration.subbands - sbc_configuration.block_length

    while (media_tracker[device].samples_ready >= num_audio_samples_per_sbc_buffer
        && (media_tracker[device].max_media_payload_size - media_tracker[device].sbc_storage_count) >= btstack_sbc_encoder_sbc_buffer_length())
    {

        produce_audio(pcm_frame, num_audio_samples_per_sbc_buffer);
        btstack_sbc_encoder_process_data(pcm_frame);
        
        sbc_frame_size = btstack_sbc_encoder_sbc_buffer_length();
        sbc_frame = btstack_sbc_encoder_sbc_buffer();

        total_num_bytes_read += num_audio_samples_per_sbc_buffer;
        memcpy(&media_tracker[device].sbc_storage[media_tracker[device].sbc_storage_count], sbc_frame, sbc_frame_size);
        media_tracker[device].sbc_storage_count += sbc_frame_size;
        media_tracker[device].samples_ready -= num_audio_samples_per_sbc_buffer;
    }
    return total_num_bytes_read;
}

static void my_a2dp_handle_timeout(uint8_t device)
{
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t update_period_ms = AUDIO_TIMEOUT_MS;
    uint32_t num_samples;
    uint8_t localStream = 0;
    //avdtp_context_t * p;
    //debug through GPIO pin
    if(device)
    {
        gpio_set_level(GPIO_OUTPUT_IO_1,1);
    }
    else
    {
        gpio_set_level(GPIO_OUTPUT_IO_0,1);
    }

//    if(idx_handler.stream_both)
//    {
//        localStream = device; //overwrite the stream device idx by the timer device selection
//    }
//    else
    {
        //localStream = idx_handler.stream;
    }

    if (media_tracker[localStream].time_audio_data_sent > 0) //when the a2dp_demo_timer_stop() function is called the variable will be resetted
    {
        update_period_ms = now - media_tracker[localStream].time_audio_data_sent;
    } 

    //num_samples = (update_period_ms * A2DP_SAMPLE_RATE_CALC) >> A2DP_SAMPLE_RATE_SHIFT_DIV; // pl: 5*44100 / 1000 = 220
    num_samples = (update_period_ms * A2DP_SAMPLE_RATE) / 1000; // pl: 5*44100 / 1000 = 220
    media_tracker[localStream].acc_num_missed_samples += (update_period_ms * A2DP_SAMPLE_RATE) % 1000;
    
    while (media_tracker[localStream].acc_num_missed_samples >= 1000){
        num_samples++;
        media_tracker[localStream].acc_num_missed_samples -= 1000;
    }
    media_tracker[localStream].time_audio_data_sent = now;
    media_tracker[localStream].samples_ready += num_samples;

    if (media_tracker[localStream].sbc_ready_to_send)
    {
        //printf("sbc_ready_to_send... \n");
        return;
    }

    my_a2dp_fill_sbc_audio_buffer(localStream); //if alter is not enabled then fill the buffer every time the handler is called

    if ((media_tracker[localStream].sbc_storage_count + btstack_sbc_encoder_sbc_buffer_length()) > media_tracker[localStream].max_media_payload_size)
    {
        // schedule sending
        //printf("API - ready \n");
        if(idx_handler.stream_both) //if stream_both is set then here starts the first request for the first connected device, the second request will start immediatly after the first one is served -> A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW
        {
            media_tracker[localStream].sbc_ready_to_send = 1;
            local_stream_endpoint = avdtp_stream_endpoint_for_seid(media_tracker[0].local_seid, a2dp_get_source_context());
            local_stream_endpoint->send_stream = 1;
            local_stream_endpoint->connection->wait_to_send_initiator = 1;
            l2cap_request_can_send_now_event(l2cap_media_cid[0]);
        }
        else
        {
            media_tracker[localStream].sbc_ready_to_send = 1;
            local_stream_endpoint = avdtp_stream_endpoint_for_seid(media_tracker[idx_handler.stream].local_seid, a2dp_get_source_context());
            local_stream_endpoint->send_stream = 1;
            local_stream_endpoint->connection->wait_to_send_initiator = 1;
            l2cap_request_can_send_now_event(l2cap_media_cid[idx_handler.stream]);
        }

    }
    else
    {
        //printf("API - max_media_payload_size reached \n");
    }
    //debug through GPIO pin
    if(device)
    {
        gpio_set_level(GPIO_OUTPUT_IO_1,0);
    }
    else
    {
        gpio_set_level(GPIO_OUTPUT_IO_0,0);
    }
}

static void a2dp_demo_audio_timeout_handler_0(btstack_timer_source_t * timer){
    a2dp_media_sending_context_t * context = (a2dp_media_sending_context_t *) btstack_run_loop_get_timer_context(timer);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS);
    btstack_run_loop_add_timer(&context->audio_timer);

    my_a2dp_handle_timeout(0);
}

static void a2dp_demo_audio_timeout_handler_1(btstack_timer_source_t * timer){
    a2dp_media_sending_context_t * context = (a2dp_media_sending_context_t *) btstack_run_loop_get_timer_context(timer);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS);
    btstack_run_loop_add_timer(&context->audio_timer);

    my_a2dp_handle_timeout(1);
}

static void a2dp_demo_timer_start(a2dp_media_sending_context_t * context){
    context->max_media_payload_size = btstack_min(a2dp_max_media_payload_size(context->a2dp_cid, context->local_seid), SBC_STORAGE_SIZE);
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = 0;
    context->streaming = 1;
    btstack_run_loop_remove_timer(&context->audio_timer);
    if(idx_handler.config)
    {
        btstack_run_loop_set_timer_handler(&context->audio_timer, a2dp_demo_audio_timeout_handler_1);
    }
    else
    {
        btstack_run_loop_set_timer_handler(&context->audio_timer, a2dp_demo_audio_timeout_handler_0);
    }
    btstack_run_loop_set_timer_context(&context->audio_timer, context);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS); 
    btstack_run_loop_add_timer(&context->audio_timer);
}

static void a2dp_demo_timer_stop(a2dp_media_sending_context_t * context){
#ifdef API_DEBUG
    printf("API - in the function %s \n", __func__);
#endif
    context->time_audio_data_sent = 0;
    context->acc_num_missed_samples = 0;
    context->samples_ready = 0;
    context->streaming = 1;
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = 0;
    btstack_run_loop_remove_timer(&context->audio_timer);
} 

static void a2dp_source_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);
    uint8_t status;
    uint8_t local_seid;
    bd_addr_t address;
    uint16_t cid;
    //printf("API - in the function %s \n", __func__);
    //printf("API - hci_event_packet_get_type(packet): %d \n", hci_event_packet_get_type(packet));
    //printf("API - packet[2]: %d \n", packet[2]);


    if (packet_type != HCI_EVENT_PACKET) return;

#ifndef HAVE_BTSTACK_STDIN
    if (hci_event_packet_get_type(packet) == BTSTACK_EVENT_STATE){
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
        printf("Create AVDTP Source connection to addr %s.\n", bd_addr_to_str(&device_addr[0][0]));
        status = a2dp_source_establish_stream(&device_addr[idx_handler.config][0], media_tracker[idx_handler.config].local_seid, &media_tracker[idx_handler.config].a2dp_cid);
        if (status != ERROR_CODE_SUCCESS){
            printf("Could not perform command, status 0x%2x\n", status);
        }
        return;
    }
#endif
    if (hci_event_packet_get_type(packet) == HCI_EVENT_PIN_CODE_REQUEST) {
        printf("Pin code request - using '0000'\n");
        hci_event_pin_code_request_get_bd_addr(packet, address);
        gap_pin_code_response(address, "0000");
        return;
    }
    
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;
    switch (packet[2]){
        case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED: //first one
            a2dp_subevent_signaling_connection_established_get_bd_addr(packet, address);
            cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
            status = a2dp_subevent_signaling_connection_established_get_status(packet);

            if (status != ERROR_CODE_SUCCESS){
                printf("A2DP Source: Connection failed, status 0x%02x, cid 0x%02x, a2dp_cid 0x%02x \n", status, cid, media_tracker[idx_handler.config].a2dp_cid);
                media_tracker[idx_handler.config].a2dp_cid = 0;
                break;
            }
            media_tracker[idx_handler.config].a2dp_cid = cid;
            printf("A2DP Source: Connected to address %s, a2dp cid 0x%02x.\n", bd_addr_to_str(address), media_tracker[idx_handler.config].a2dp_cid);
            break;

         case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION:{
            printf("A2DP Source: Received SBC codec configuration.\n");
            //if(!(media_tracker[0].timer_started || media_tracker[1].timer_started))
            {
            sbc_configuration.reconfigure = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
            sbc_configuration.num_channels = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
            sbc_configuration.sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            sbc_configuration.channel_mode = a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet);
            sbc_configuration.block_length = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
            sbc_configuration.subbands = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
            sbc_configuration.allocation_method = a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);
            sbc_configuration.min_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
            sbc_configuration.max_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);
            sbc_configuration.frames_per_buffer = sbc_configuration.subbands * sbc_configuration.block_length;

            btstack_sbc_encoder_init(&sbc_encoder_state, SBC_MODE_STANDARD, 
                sbc_configuration.block_length, sbc_configuration.subbands, 
                sbc_configuration.allocation_method, sbc_configuration.sampling_frequency, 
                sbc_configuration.max_bitpool_value,
                sbc_configuration.channel_mode);
            }
//            else
//            {
//                printf("A2DP Source: SBC codec already configed.\n");
//            }
            
//             status = a2dp_source_establish_stream(device_addr[idx_handler.config][0], media_tracker[idx_handler.config].local_seid, &media_tracker[idx_handler.config].a2dp_cid);
//             if (status != ERROR_CODE_SUCCESS){
//                 printf("Could not perform command, status 0x%2x\n", status);
//             }
            break;
        }  

        case A2DP_SUBEVENT_STREAM_ESTABLISHED: //second one
            a2dp_subevent_stream_established_get_bd_addr(packet, address);
            status = a2dp_subevent_stream_established_get_status(packet);
            if (status){
                printf("A2DP Source: Stream failed, status 0x%02x.\n", status);
                break;
            }
            local_seid = a2dp_subevent_stream_established_get_local_seid(packet);
            if (local_seid != media_tracker[idx_handler.config].local_seid){
                printf("A2DP Source: Stream failed, wrong local seid %d, expected %d.\n", local_seid, media_tracker[idx_handler.config].local_seid);
                break;    
            }
            printf("A2DP Source: Stream established, address %s, a2dp cid 0x%02x, local seid %d, remote seid %d.\n", bd_addr_to_str(address),
                media_tracker[idx_handler.config].a2dp_cid, media_tracker[idx_handler.config].local_seid, a2dp_subevent_stream_established_get_remote_seid(packet));
            printf("A2DP Source: Start playing mod, a2dp cid 0x%02x.\n", media_tracker[idx_handler.config].a2dp_cid);
            media_tracker[idx_handler.config].stream_opened = 1;
            data_source = STREAM_MOD;
            local_stream_endpoint = avdtp_stream_endpoint_for_seid(media_tracker[idx_handler.config].local_seid, a2dp_get_source_context());
            //a2dp_source_stream_endpoint_request_can_send_now(context->a2dp_cid, context->local_seid);
            //l2cap_request_can_send_now_event(media_tracker[idx_handler.stream].a2dp_cid);
            l2cap_media_cid[idx_handler.config] = local_stream_endpoint->l2cap_media_cid;
//            l2cap_media_cid[idx_handler.connected_devices] = local_stream_endpoint->l2cap_media_cid;
//            if(idx_handler.connected_devices < BT_NUM_OF_DEVICES)
//            {
//                idx_handler.connected_devices++;
//            }
            status = a2dp_source_start_stream(media_tracker[idx_handler.config].a2dp_cid, media_tracker[idx_handler.config].local_seid);
            break;

        case A2DP_SUBEVENT_STREAM_STARTED:
            play_info.status = AVRCP_PLAYBACK_STATUS_PLAYING;
            if (media_tracker[idx_handler.config].avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker[idx_handler.config].avrcp_cid, &tracks[data_source], sizeof(tracks)/sizeof(avrcp_track_t));
                avrcp_target_set_playback_status(media_tracker[idx_handler.config].avrcp_cid, AVRCP_PLAYBACK_STATUS_PLAYING);
            }
            if(!(media_tracker[0].timer_started || media_tracker[1].timer_started))
            {
                a2dp_demo_timer_start(&media_tracker[idx_handler.config]);
                //a2dp_demo_timer_start(&media_tracker[0]);
            }
            media_tracker[idx_handler.config].timer_started = 1;
            printf("A2DP Source: Stream started.\n");
            break;

        case A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW:
            cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
            //printf("Initiator cid: 0x%02x \n", cid);
            if(l2cap_media_cid[idx_handler.stream] != cid)
            {
                if(idx_handler.stream_both)
                {
                    idx_handler.stream ^= 0x01;
                }
            }
            a2dp_demo_send_media_packet();

            break;        

        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            play_info.status = AVRCP_PLAYBACK_STATUS_PAUSED;
            if (media_tracker[idx_handler.config].avrcp_cid){
                avrcp_target_set_playback_status(media_tracker[idx_handler.config].avrcp_cid, AVRCP_PLAYBACK_STATUS_PAUSED);
            }
            printf("A2DP Source: Stream paused.\n");
            a2dp_demo_timer_stop(&media_tracker[idx_handler.config]);
            break;

        case A2DP_SUBEVENT_STREAM_RELEASED:
            play_info.status = AVRCP_PLAYBACK_STATUS_STOPPED;
            cid = a2dp_subevent_stream_released_get_a2dp_cid(packet);
            if (cid == media_tracker[idx_handler.config].a2dp_cid) {
                media_tracker[idx_handler.config].stream_opened = 0;
                printf("A2DP Source: Stream released.\n");
            }
            if (media_tracker[idx_handler.config].avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker[idx_handler.config].avrcp_cid, NULL, sizeof(tracks)/sizeof(avrcp_track_t));
                avrcp_target_set_playback_status(media_tracker[idx_handler.config].avrcp_cid, AVRCP_PLAYBACK_STATUS_STOPPED);
            }
            a2dp_demo_timer_stop(&media_tracker[idx_handler.config]);
            break;
        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            cid = a2dp_subevent_signaling_connection_released_get_a2dp_cid(packet);
            if (cid == media_tracker[idx_handler.config].a2dp_cid) {
                media_tracker[idx_handler.config].avrcp_cid = 0;
                media_tracker[idx_handler.config].a2dp_cid = 0;
                printf("A2DP Source: Signaling released.\n\n");
            }
            break;
        default:
            break; 
    }
}

static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    bd_addr_t event_addr;
    uint16_t local_cid;
    uint8_t  status = ERROR_CODE_SUCCESS;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    
    switch (packet[2]){
        case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED: {
            local_cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
            // if (avrcp_cid != 0 && avrcp_cid != local_cid) {
            //     printf("AVRCP Target: Connection failed, expected 0x%02X l2cap cid, received 0x%02X\n", avrcp_cid, local_cid);
            //     return;
            // }
            // if (avrcp_cid != local_cid) break;
            
            status = avrcp_subevent_connection_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                printf("AVRCP Target: Connection failed, status 0x%02x\n", status);
                return;
            }
            media_tracker[idx_handler.config].avrcp_cid = local_cid;
            avrcp_subevent_connection_established_get_bd_addr(packet, event_addr);
            printf("AVRCP Target: Connected to %s, avrcp_cid 0x%02x\n", bd_addr_to_str(event_addr), local_cid);
            
            avrcp_target_set_now_playing_info(media_tracker[idx_handler.config].avrcp_cid, NULL, sizeof(tracks)/sizeof(avrcp_track_t));
            avrcp_target_set_unit_info(media_tracker[idx_handler.config].avrcp_cid, AVRCP_SUBUNIT_TYPE_AUDIO, company_id);
            avrcp_target_set_subunit_info(media_tracker[idx_handler.config].avrcp_cid, AVRCP_SUBUNIT_TYPE_AUDIO, (uint8_t *)subunit_info, sizeof(subunit_info));
            return;
        }
        
        case AVRCP_SUBEVENT_EVENT_IDS_QUERY:
            status = avrcp_target_supported_events(media_tracker[idx_handler.config].avrcp_cid, events_num, events, sizeof(events));
            break;
        case AVRCP_SUBEVENT_COMPANY_IDS_QUERY:
            status = avrcp_target_supported_companies(media_tracker[idx_handler.config].avrcp_cid, companies_num, companies, sizeof(companies));
            break;
        case AVRCP_SUBEVENT_PLAY_STATUS_QUERY:
            status = avrcp_target_play_status(media_tracker[idx_handler.config].avrcp_cid, play_info.song_length_ms, play_info.song_position_ms, play_info.status);
            break;
        // case AVRCP_SUBEVENT_NOW_PLAYING_INFO_QUERY:
        //     status = avrcp_target_now_playing_info(avrcp_cid);
        //     break;
        case AVRCP_SUBEVENT_OPERATION:{
            avrcp_operation_id_t operation_id = avrcp_subevent_operation_get_operation_id(packet);
            switch (operation_id){
                case AVRCP_OPERATION_ID_PLAY:
                    printf("AVRCP Target: PLAY\n");
                    status = a2dp_source_start_stream(media_tracker[idx_handler.config].a2dp_cid, media_tracker[idx_handler.config].local_seid);
                    break;
                case AVRCP_OPERATION_ID_PAUSE:
                    printf("AVRCP Target: PAUSE\n");
                    status = a2dp_source_pause_stream(media_tracker[idx_handler.config].a2dp_cid, media_tracker[idx_handler.config].local_seid);
                    break;
                case AVRCP_OPERATION_ID_STOP:
                    printf("AVRCP Target: STOP\n");
                    status = a2dp_source_disconnect(media_tracker[idx_handler.config].a2dp_cid);
                    break;
                default:
                    printf("AVRCP Target: operation 0x%2x is not handled\n", operation_id);
                    return;
            }
            break;
        }
        case AVRCP_SUBEVENT_CONNECTION_RELEASED:
            printf("AVRCP Target: Disconnected, avrcp_cid 0x%02x\n", avrcp_subevent_connection_released_get_avrcp_cid(packet));
            media_tracker[idx_handler.config].avrcp_cid = 0;
            return;
        default:
            break;
    }

    if (status != ERROR_CODE_SUCCESS){
        printf("Responding to event 0x%02x failed with status 0x%02x\n", packet[2], status);
    }
}

#ifdef HAVE_BTSTACK_STDIN
static void show_usage(void){
    bd_addr_t      iut_address;
    gap_local_bd_addr(iut_address);
    printf("\n--- Bluetooth  A2DP Source/AVRCP Target Demo %s ---\n", bd_addr_to_str(iut_address));
    printf("0      - Select 0. device as active device \n");
    printf("1      - Select 1. device as active device \n");
    printf("2 - The selected stream is 0.\n");
    printf("3 - The selected stream is 1.\n");

    if(idx_handler.config)
    {
        printf("b      - AVDTP Source create connection to addr %s\n", device_addr_string_1);
    }
    else
    {
        printf("b      - AVDTP Source create connection to addr %s\n", device_addr_string_0);
    }
    printf("B      - AVDTP Source disconnect\n");
    if(idx_handler.config)
    {
        printf("c      - AVRCP Target create connection to addr %s\n", device_addr_string_1);
    }
    else
    {
        printf("c      - AVRCP Source create connection to addr %s\n", device_addr_string_0);
    }
    printf("C      - AVRCP Target disconnect\n");

    printf("m - stream to both device \n");
    printf("n - a2dp_source_stream_endpoint_request_can_send_now() \n");
    printf("N - a2dp_source_start_stream() \n");
    printf("D - disable discovery \n");
    printf("E - enable discovery \n");
    printf("t - start timer \n");
    printf("T - stop timer \n");
    printf("I - A2DP_SUBEVENT_STREAM_ESTABLISHED \n");
    printf("l - start timer for both sinks \n");
    printf("a - alter playment device \n");

    printf("x      - start streaming sine\n");
#ifdef USE_SD_CARD_AUDIO
    printf("%z - Play from sd card.\n");
#endif //USE_SD_CARD_AUDIO
    printf("s      - get media_tracker status \n");
    if (hxcmod_initialized){
        printf("z      - start streaming '%s'\n", mod_name);
    }
    printf("p      - pause streaming\n");

    printf("\n--- Bluetooth  AVRCP Target Commands %s ---\n", bd_addr_to_str(iut_address));
    printf("---\n");
}

static void stdin_process(char cmd){
    uint16_t i;
    uint8_t status = ERROR_CODE_SUCCESS;
    avdtp_context_t * tempStatus;
    switch (cmd){
        case '0':
            idx_handler.config = 0;
            printf("%c - The selected device is %d, addr %s, cid 0x%02x.\n", cmd, &device_addr[idx_handler.config][0], bd_addr_to_str(&device_addr[idx_handler.config][0]), media_tracker[idx_handler.config].a2dp_cid);
            break;
        case '1':
            idx_handler.config = 1;
            printf("%c - The selected device is %d, addr %s, cid 0x%02x.\n", cmd, &device_addr[idx_handler.config][0], bd_addr_to_str(&device_addr[idx_handler.config][0]), media_tracker[idx_handler.config].a2dp_cid);
            break;
        case '2':
//            if(media_tracker[0].timer_started)
//            {
//                a2dp_demo_timer_stop(&media_tracker[0]);
//            }
//            if(media_tracker[1].timer_started)
//            {
//                a2dp_demo_timer_stop(&media_tracker[1]);
//            }
            idx_handler.stream = 0;
            //a2dp_demo_timer_start(&media_tracker[idx_handler.config]);
            printf("%c - The selected stream is %d.\n",cmd, idx_handler.stream);
            break;
        case '3':
//            if(media_tracker[0].timer_started)
//            {
//                a2dp_demo_timer_stop(&media_tracker[0]);
//            }
//            if(media_tracker[1].timer_started)
//            {
//                a2dp_demo_timer_stop(&media_tracker[1]);
//            }
            idx_handler.stream = 1;
            //a2dp_demo_timer_start(&media_tracker[idx_handler.config]);
            printf("%c - The selected stream is %d.\n",cmd, idx_handler.stream);
            break;
        case 'b':
            if(media_tracker[0].timer_started)
            {
                printf("A2DP Source: Stream timer paused for idx 0.\n");
                a2dp_demo_timer_stop(&media_tracker[0]);
                media_tracker[0].timer_started = 0;
            }
            if(media_tracker[1].timer_started)
            {
                printf("A2DP Source: Stream timer paused for idx 1.\n");
                a2dp_demo_timer_stop(&media_tracker[1]);
                media_tracker[1].timer_started = 0;
            }
            status = a2dp_source_establish_stream(&device_addr[idx_handler.config][0], media_tracker[idx_handler.config].local_seid, &media_tracker[idx_handler.config].a2dp_cid);
            printf("%c - Create AVDTP Source connection to addr %s, cid 0x%02x.\n", cmd, bd_addr_to_str(&device_addr[idx_handler.config][0]), media_tracker[idx_handler.config].a2dp_cid);
            break;
        case 'B':
            printf("%c - AVDTP Source Disconnect from cid 0x%2x\n", cmd, media_tracker[idx_handler.config].a2dp_cid);
            status = a2dp_source_disconnect(media_tracker[idx_handler.config].a2dp_cid);
            break;
        case 'c':
            printf("%c - Create AVRCP Target connection to addr %s.\n", cmd, bd_addr_to_str(&device_addr[idx_handler.config][0]));
            status = avrcp_target_connect(&device_addr[idx_handler.config][0], &media_tracker[idx_handler.config].avrcp_cid);
            break;
        case 'C':
            printf("%c - AVRCP Target disconnect\n", cmd);
            status = avrcp_target_disconnect(media_tracker[idx_handler.config].avrcp_cid);
            break;
        case 't':
            a2dp_demo_timer_start(&media_tracker[idx_handler.config]);
            printf("%c - Timer started \n",cmd);
            break;
        case 'T':
            a2dp_demo_timer_stop(&media_tracker[idx_handler.config]);
            printf("%c - Timer stopped \n",cmd);
            break;
        case 'n':
            printf("%c - a2dp_source_stream_endpoint_request_can_send_now() \n",cmd);
            a2dp_source_stream_endpoint_request_can_send_now(media_tracker[idx_handler.stream].a2dp_cid, media_tracker[idx_handler.stream].local_seid);
            break;
        case 'l':
            printf("%c - timer started for both sinks \n",cmd);
            a2dp_demo_timer_start(&media_tracker[0]);
            media_tracker[1].streaming = 1;
            //a2dp_demo_timer_start(&media_tracker[1]);
            break;
        case 'm':
            if(idx_handler.stream_both)
            {
                printf("%c - stream to both device stop \n",cmd);
                idx_handler.stream_both = 0x00;
            }
            else
            {
                if(media_tracker[0].timer_started)
                {
                    a2dp_demo_timer_stop(&media_tracker[0]);
                }
                if(media_tracker[1].timer_started)
                {
                    a2dp_demo_timer_stop(&media_tracker[1]);
                }
                idx_handler.config = 0;
                idx_handler.stream = 0;
                deviceIdx = 0;
                idx_handler.stream_both = 0x01;
                a2dp_demo_timer_start(&media_tracker[idx_handler.config]);

                printf("%c - stream to both device start \n",cmd);
            }
            break;
        case 'w':
            avrcp_target_set_now_playing_info(media_tracker[idx_handler.config].avrcp_cid, &tracks[data_source], sizeof(tracks)/sizeof(avrcp_track_t));
            avrcp_target_set_playback_status(media_tracker[idx_handler.config].avrcp_cid, AVRCP_PLAYBACK_STATUS_PLAYING);
        break;
        case 's':
            tempStatus = a2dp_get_source_context();
            printf("%c - status requested \n",cmd);
            printf("--sbc configuration--\n");
            printf("num_channels: %d \n", sbc_configuration.num_channels);
            printf("sampling_frequency: %d \n", sbc_configuration.sampling_frequency);
            printf("channel_mode: %d \n", sbc_configuration.channel_mode);
            printf("subbands: %d \n", sbc_configuration.subbands);
            printf("allocation_method: %d \n", sbc_configuration.allocation_method);
            printf("min_bitpool_value: %d \n", sbc_configuration.min_bitpool_value);
            printf("max_bitpool_value: %d \n", sbc_configuration.max_bitpool_value);
            printf("frames_per_buffer: %d \n", sbc_configuration.frames_per_buffer);
            printf("-------\n");

            local_stream_endpoint = avdtp_stream_endpoint_for_seid(media_tracker[0].local_seid, tempStatus);
            printf("media_tracker[0].local_seid = %d \n",media_tracker[0].local_seid);
            printf("media_tracker[0].avrcp_cid = %d \n",media_tracker[0].avrcp_cid);
            if(local_stream_endpoint)
            {
                printf("max_media_payload_size: %d \n", a2dp_max_media_payload_size(media_tracker[0].a2dp_cid, media_tracker[0].local_seid));
                printf("local_stream_endpoint->l2cap_media_cid = 0x%02x \n",local_stream_endpoint->l2cap_media_cid);
                printf("local_stream_endpoint->l2cap_recovery_cid = 0x%02x \n",local_stream_endpoint->l2cap_recovery_cid);
                printf("local_stream_endpoint->l2cap_reporting_cid = 0x%02x \n",local_stream_endpoint->l2cap_reporting_cid);
                printf("local_stream_endpoint->state = %d \n",local_stream_endpoint->state);
                printf("local_stream_endpoint->a2dp_state = %d \n",local_stream_endpoint->a2dp_state);
                printf("local_stream_endpoint->start_stream = %d \n",local_stream_endpoint->start_stream);
                printf("local_stream_endpoint->stop_stream = %d \n",local_stream_endpoint->stop_stream);
                printf("local_stream_endpoint->abort_stream = %d \n",local_stream_endpoint->abort_stream);
                printf("local_stream_endpoint->suspend_stream = %d \n",local_stream_endpoint->suspend_stream);
                printf("local_stream_endpoint->send_stream = %d \n",local_stream_endpoint->send_stream);
                printf("local_stream_endpoint->initiator_config_state = %d \n",local_stream_endpoint->initiator_config_state);
                if(local_stream_endpoint->connection)
                {
                    printf("local_stream_endpoint->connection->avdtp_cid = %d \n",local_stream_endpoint->connection->avdtp_cid);
                    printf("local_stream_endpoint->connection->state = %d \n",local_stream_endpoint->connection->state);
                    printf("local_stream_endpoint->connection->l2cap_signaling_cid = %d \n",local_stream_endpoint->connection->l2cap_signaling_cid);
                }
            }
            local_stream_endpoint = avdtp_stream_endpoint_for_seid(media_tracker[1].local_seid, tempStatus);
            printf("media_tracker[1].local_seid = %d \n",media_tracker[1].local_seid);
            printf("media_tracker[1].avrcp_cid = %d \n",media_tracker[1].avrcp_cid);
            if(local_stream_endpoint)
            {
                printf("max_media_payload_size: %d \n", a2dp_max_media_payload_size(media_tracker[1].a2dp_cid, media_tracker[1].local_seid));
                printf("local_stream_endpoint->l2cap_media_cid = 0x%02x \n",local_stream_endpoint->l2cap_media_cid);
                printf("local_stream_endpoint->l2cap_recovery_cid = 0x%02x \n",local_stream_endpoint->l2cap_recovery_cid);
                printf("local_stream_endpoint->l2cap_reporting_cid = 0x%02x \n",local_stream_endpoint->l2cap_reporting_cid);
                printf("local_stream_endpoint->state = %d \n",local_stream_endpoint->state);
                printf("local_stream_endpoint->a2dp_state = %d \n",local_stream_endpoint->a2dp_state);
                printf("local_stream_endpoint->start_stream = %d \n",local_stream_endpoint->start_stream);
                printf("local_stream_endpoint->stop_stream = %d \n",local_stream_endpoint->stop_stream);
                printf("local_stream_endpoint->abort_stream = %d \n",local_stream_endpoint->abort_stream);
                printf("local_stream_endpoint->suspend_stream = %d \n",local_stream_endpoint->suspend_stream);
                printf("local_stream_endpoint->send_stream = %d \n",local_stream_endpoint->send_stream);
                printf("local_stream_endpoint->initiator_config_state = %d \n",local_stream_endpoint->initiator_config_state);
                if(local_stream_endpoint->connection)
                {
                    printf("local_stream_endpoint->connection->avdtp_cid = %d \n",local_stream_endpoint->connection->avdtp_cid);
                    printf("local_stream_endpoint->connection->state = %d \n",local_stream_endpoint->connection->state);
                    printf("local_stream_endpoint->connection->l2cap_signaling_cid = %d \n",local_stream_endpoint->connection->l2cap_signaling_cid);
                }
            }
            printf("stored l2cap_media_cid[0] = %d \n",l2cap_media_cid[0]);
            printf("stored l2cap_media_cid[1] = %d \n",l2cap_media_cid[1]);
            break;
        case 'D':
            printf("%c - disable discovery \n",cmd);
            gap_discoverable_control(0);
            gap_connectable_control(0);
            break;
        case 'E':
            printf("%c - enable discovery \n",cmd);
            gap_discoverable_control(1);
            gap_connectable_control(1);
            break;
        case 'a':
            printf("%c - enable alternate \n",cmd);
            idx_handler.stream = 0;
            idx_handler.config = 0;
            idx_handler.alter ^= 0x01;
            break;
        case 'q':
            idx_handler.debug = 0x01;
            break;
        case '\n':
        case '\r':
            break;

        case 'x':
            if (media_tracker[idx_handler.config].avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker[idx_handler.config].avrcp_cid, &tracks[data_source], sizeof(tracks)/sizeof(avrcp_track_t));
            }
            printf("%c - Play sine.\n", cmd);
            data_source = STREAM_SINE;
            if (!media_tracker[idx_handler.config].stream_opened) break;
            status = a2dp_source_start_stream(media_tracker[idx_handler.config].a2dp_cid, media_tracker[idx_handler.config].local_seid);
            break;
        case 'z':
            if (media_tracker[idx_handler.config].avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker[idx_handler.config].avrcp_cid, &tracks[data_source], sizeof(tracks)/sizeof(avrcp_track_t));
            }
            printf("%c - Play mod.\n", cmd);
            data_source = STREAM_MOD;
            if (!media_tracker[idx_handler.config].stream_opened) break;
            status = a2dp_source_start_stream(media_tracker[idx_handler.config].a2dp_cid, media_tracker[idx_handler.config].local_seid);
            break;
#ifdef USE_SD_CARD_AUDIO
        case 'Q':
            printf("%c - Print dataset from SD card:\n", cmd);
            read(m_sample, (unsigned short *) &pcm_frame[0], 128); //read from sd card len number of samples to the data buffer
            for(i = 0;i<64;i++)
            {
                printf("%d , ",pcm_frame[i]);
            }
            printf("---------");
            break;
        case 'y':
            if (media_tracker[idx_handler.config].avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker[idx_handler.config].avrcp_cid, &tracks[data_source], sizeof(tracks)/sizeof(avrcp_track_t));
            }
            printf("%c - Play from sd card.\n", cmd);
            data_source = STREAM_FROM_SD;
            if (!media_tracker[idx_handler.config].stream_opened) break;
            status = a2dp_source_start_stream(media_tracker[idx_handler.config].a2dp_cid, media_tracker[idx_handler.config].local_seid);
            break;
#endif //USE_SD_CARD_AUDIO
        case 'p':
            if (!media_tracker[idx_handler.config].stream_opened) break;
            printf("%c - Pause stream.\n", cmd);
            status = a2dp_source_pause_stream(media_tracker[idx_handler.config].a2dp_cid, media_tracker[idx_handler.config].local_seid);
            break;
        
        default:
            show_usage();
            return;
    }
    if (status != ERROR_CODE_SUCCESS){
        printf("Could not perform command \'%c\', status 0x%2x\n", cmd, status);
    }
}
#endif

#ifdef USE_SD_CARD_AUDIO
int bt_api_init_sd(void)
{
    esp_err_t ret = 0;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes
    gpio_pullup_en(GPIO_NUM_15);
    gpio_pullup_en(GPIO_NUM_2);
    gpio_pullup_en(GPIO_NUM_4);
    gpio_pullup_en(GPIO_NUM_12);
    gpio_pullup_en(GPIO_NUM_13);

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        return 1;
    }

    sdmmc_card_print_info(stdout, card);
    m_sample = open("/sdcard/aud04.wav", O_RDONLY);

    return 0;
}
#endif //USE_SD_CARD_AUDIO

int btstack_main(int argc, const char * argv[]);

void api_gpio_init()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

int btstack_main(int argc, const char * argv[]){
    (void)argc;
    (void)argv;
    int err;
#ifdef USE_SD_CARD_AUDIO
    err = bt_api_init_sd();
    if (err) return err;
#endif //USE_SD_CARD_AUDIO
    api_gpio_init(); //init GPIO
    err = a2dp_source_and_avrcp_services_init();
    if (err) return err;
    // turn on!
    hci_power_control(HCI_POWER_ON);
    return 0;
}
/* EXAMPLE_END */
