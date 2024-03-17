#include<stdio.h>
#include<string.h>

#define BUFFER_SIZE 512

#define CRSF_SYNC_BYTE 0XC8
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0XC8
// frame types taken from: https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf_protocol.h#L37
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_COMMAND = 0x32,
// 0x8F - was received, unclear what frametype is @todo search both tx (express lrs) and rx (betaflight) sources
#define CRSF_FRAMETYPE_HZ 0x8F
#define CRSF_FRAMETYPE_ATTITUDE 0x1E
char verbose = 0;

/*

all received frametypes:
0x00
0x01  CRSF_DISPLAYPORT_SUBCMD_UPDATE = 0x01, // transmit displayport buffer to remote
0x1E CRSF_FRAMETYPE_ATTITUDE = 0x1E,!! need to implement @todo
0x02   CRSF_FRAMETYPE_GPS = 0x02,
0x20
0x21 CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
0x03 ?? CRSF_DISPLAYPORT_SUBCMD_OPEN = 0x03,  // client request to open cms menu
0x04 ?? CRSF_DISPLAYPORT_SUBCMD_CLOSE = 0x04,  // client request to close cms menu
0x60
0x7D  CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D, // displayport control command
0x08   CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
0x0B   CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
0x0D
0xE0
*/

// idea source https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c#L110
struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

/*
 * https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c#L151
 * 0x14 Link statistics
 * Payload (10 bytes):
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */
typedef struct crsfPayloadLinkstatistics_s {
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} crsfLinkStatistics_t;

typedef struct crsfPayloadAttitude_s {
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
} crsfAttitude_t;

typedef struct crsfPacket_s {
    uint8_t device_id;
    uint8_t frame_size;
    uint8_t frame_type;
    uint16_t received_channels[16];
    uint16_t telemetry_channels[16];
    uint8_t crc;
} CRSF_Packet;

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

int calculateCRC(uint8_t * data, uint16_t position, uint8_t frame_size)
{
    // CRC includes type and payload
    uint8_t crc = 0;//crc8_dvb_s2(0, device_id);
    //crc = crc8_dvb_s2(crc, frame_size);

    for (int ii = position; ii < position + frame_size - 1; ++ii) {
        crc = crc8_dvb_s2(crc, data[ii]);
        //printf("crc calc, ii=%d, data[ii]=%x, crc =%x\n", ii, data[ii], crc);
    }

    return crc;
}

void processCrsfFrame(uint8_t * data, uint8_t device_id, uint8_t frame_size, uint8_t crc,
   uint32_t crc_failures_count, struct crsfPacket_s *crsf_packet, 
   struct crsfPayloadLinkstatistics_s* crsf_link_statistics, 
   struct crsfPayloadAttitude_s * crsf_attitude
   )
{
    uint8_t frame_type = data[0];
    crsf_packet->device_id = device_id;
    crsf_packet->frame_size = frame_size;
    crsf_packet->frame_type = frame_type;
    crsf_packet->crc = crc;

    for(int i = 0; i < 16; i++) {
        crsf_packet->received_channels[i] = 0;
        crsf_packet->telemetry_channels[i] = 0;
    }

    switch(frame_type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
            struct crsfPayloadRcChannelsPacked_s * crsfChannels = (struct crsfPayloadRcChannelsPacked_s *) (data + 1);
            crsf_packet->received_channels[0] = crsfChannels->chan0;
            crsf_packet->received_channels[1] = crsfChannels->chan1;
            crsf_packet->received_channels[2] = crsfChannels->chan2;
            crsf_packet->received_channels[3] = crsfChannels->chan3;
            crsf_packet->received_channels[4] = crsfChannels->chan4;
            crsf_packet->received_channels[5] = crsfChannels->chan5;
            crsf_packet->received_channels[6] = crsfChannels->chan6;
            crsf_packet->received_channels[7] = crsfChannels->chan7;
            crsf_packet->received_channels[8] = crsfChannels->chan8;
            crsf_packet->received_channels[9] = crsfChannels->chan9;
            crsf_packet->received_channels[10] = crsfChannels->chan10;
            crsf_packet->received_channels[11] = crsfChannels->chan11;
            crsf_packet->received_channels[12] = crsfChannels->chan12;
            crsf_packet->received_channels[13] = crsfChannels->chan13;
            crsf_packet->received_channels[14] = crsfChannels->chan14;
            crsf_packet->received_channels[15] = crsfChannels->chan15;
            break;
        }

        case CRSF_FRAMETYPE_LINK_STATISTICS: {
            memcpy(crsf_link_statistics, (void *)data + 1, sizeof(struct crsfPayloadLinkstatistics_s));
            break;
        }

        case CRSF_FRAMETYPE_ATTITUDE: {
            if (frame_size != 8) {
                return;
            }

            struct crsfPayloadAttitude_s * crsfAttitude_p = (struct crsfPayloadAttitude_s *)(data + 1);
            crsf_attitude->pitch = crsfAttitude_p->pitch;
            crsf_attitude->roll = crsfAttitude_p->roll;
            crsf_attitude->yaw = crsfAttitude_p->yaw;
            break;
        }
//        default: // undefined frame
    }
    if (1) {
        printf("Received chanels:");
        for (int i = 0; i < 16; i++) {
            printf("Ch: %d, value: %d\n", i, crsf_packet->received_channels[i]);
        }

        printf("uplink_RSSI_1: %d\n", (int) crsf_link_statistics->uplink_RSSI_1);
        printf("uplink_RSSI_2: %d\n", (int) crsf_link_statistics->uplink_RSSI_2);
        printf("uplink_Link_quality: %d\n", (int) crsf_link_statistics->uplink_Link_quality);
        printf("uplink_SNR: %d\n", (int) crsf_link_statistics->uplink_SNR);
        printf("active_antenna: %d\n", (int) crsf_link_statistics->active_antenna);
        printf("rf_Mode: %d,", (int) crsf_link_statistics->rf_Mode);
        printf("uplink_TX_Power: %d\n", (int) crsf_link_statistics->uplink_TX_Power);
        printf("downlink_RSSI: %d\n", (int) crsf_link_statistics->downlink_RSSI);
        printf("downlink_Link_quality: %d\n", (int) crsf_link_statistics->downlink_Link_quality);
        printf("downlink_SNR: %d\n", (int) crsf_link_statistics->downlink_SNR);
        printf("pitch: %d\n", (int) crsf_attitude->pitch);
        printf("roll: %d\n", (int) crsf_attitude->roll);
        printf("yaw: %d\n", (int) crsf_attitude->yaw);
        printf("crc_failures_count: %d\n", (int) crc_failures_count);
        printf("tick: %d\n", (int)HAL_GetTick());
    }
}

int data_read(uint8_t * data, int n, int sourceId);

int readFromSource(int sourceId, uint8_t * data, uint16_t buffer_position, uint16_t previously_read, uint8_t n)
{
    size_t bytesRead;

    if (buffer_position + n > BUFFER_SIZE) {
        if (verbose > 2) 
          printf("memcpy\n");
        memcpy((void*)data, (void*) (data + buffer_position), BUFFER_SIZE - buffer_position);
        previously_read = BUFFER_SIZE - buffer_position;
        buffer_position = 0;

        if (verbose > 3) {
          printf("All Buffer after memcpy:\n");
          for(int ii = 0; ii < BUFFER_SIZE; ii++) {
              printf("ii = %d, ", (int)ii);
              printf("data[ii] = %X\n", (int)data[ii]);
          }
        }


    }

    if (buffer_position + n > previously_read) {
        bytesRead = data_read(data + previously_read, buffer_position + n - previously_read, sourceId);

        if (verbose > 2)
          printf("read %d bytes, starting from %d\n", (int)bytesRead, (int)previously_read);

        if (bytesRead != sizeof(uint8_t) * (buffer_position + n - previously_read)) {
            if (verbose > 2)
              printf("Error reading the data\n");

            return -1;
        }

        previously_read = previously_read + bytesRead;
    } else {
        if (verbose > 2)
          printf("No reads, data is available\n");
    }

    return previously_read;
}

void clear_link_statistics(struct crsfPayloadLinkstatistics_s* crsf_link_statistics) 
{
	crsf_link_statistics->uplink_RSSI_1 = 0;
    crsf_link_statistics->uplink_RSSI_2 = 0;
    crsf_link_statistics->uplink_Link_quality = 0;
    crsf_link_statistics->uplink_SNR = 0;
    crsf_link_statistics->active_antenna = 0;
    crsf_link_statistics->rf_Mode = 0;
    crsf_link_statistics->uplink_TX_Power = 0;
    crsf_link_statistics->downlink_RSSI = 0;
    crsf_link_statistics->downlink_Link_quality = 0;
    crsf_link_statistics->downlink_SNR = 0;
}

void clear_attitude(struct crsfPayloadAttitude_s * crsf_attitude)
{
	crsf_attitude->pitch = 0;
	crsf_attitude->roll = 0;
	crsf_attitude->yaw = 0;
}