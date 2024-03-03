#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define CRSF_SYNC_BYTE 0XC8
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0XC8
// frame types taken from: https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf_protocol.h#L37
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_COMMAND = 0x32,
// 0x8F - was received, unclear what frametype is @todo search both tx (express lrs) and rx (betaflight) sources
#define CRSF_FRAMETYPE_HZ 0x8F
#define CRSF_FRAMETYPE_ATTITUDE 0x1E

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

#define BUFFER_SIZE 512
#define DUMP_DATA_POSITION 0x3A00

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

crsfLinkStatistics_t CRSF_LinkStatistics;

typedef struct crsfPayloadAttitude_s {
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
} crsfAttitude_t;
struct crsfPayloadAttitude_s CRSF_Attitude;

typedef struct {
    uint8_t device_id;
    uint8_t frame_size;
    uint8_t frame_type;
    uint16_t received_channels[16];
    uint16_t telemetry_channels[16];
    uint8_t crc;
} CRSF_Packet;

CRSF_Packet crsf_packet;

// idea source https://github.com/betaflight/betaflight/blob/4.0-maintenance/src/main/common/crc.c#L62

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
        printf("crc calc, ii=%d, data[ii]=%x, crc =%x\n", ii, data[ii], crc);
    }

    return crc;
}

void processCrsfFrame(uint8_t * data, uint8_t device_id, uint8_t frame_size, uint8_t crc, FILE * csvFile, uint32_t crc_failures_count, uint32_t dump_position)
{
    uint8_t frame_type = data[0];
    crsf_packet.device_id = device_id;
    crsf_packet.frame_size = frame_size;
    crsf_packet.frame_type = frame_type;
    crsf_packet.crc = crc;

    for(int i = 0; i < 16; i++) {
        crsf_packet.received_channels[i] = 0;
        crsf_packet.telemetry_channels[i] = 0;
    }

    switch(frame_type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
            struct crsfPayloadRcChannelsPacked_s * crsfChannels = (struct crsfPayloadRcChannelsPacked_s *) (data + 1);
            crsf_packet.received_channels[0] = crsfChannels->chan0;
            crsf_packet.received_channels[1] = crsfChannels->chan1;
            crsf_packet.received_channels[2] = crsfChannels->chan2;
            crsf_packet.received_channels[3] = crsfChannels->chan3;
            crsf_packet.received_channels[4] = crsfChannels->chan4;
            crsf_packet.received_channels[5] = crsfChannels->chan5;
            crsf_packet.received_channels[6] = crsfChannels->chan6;
            crsf_packet.received_channels[7] = crsfChannels->chan7;
            crsf_packet.received_channels[8] = crsfChannels->chan8;
            crsf_packet.received_channels[9] = crsfChannels->chan9;
            crsf_packet.received_channels[10] = crsfChannels->chan10;
            crsf_packet.received_channels[11] = crsfChannels->chan11;
            crsf_packet.received_channels[12] = crsfChannels->chan12;
            crsf_packet.received_channels[13] = crsfChannels->chan13;
            crsf_packet.received_channels[14] = crsfChannels->chan14;
            crsf_packet.received_channels[15] = crsfChannels->chan15;
            break;
        }

        case CRSF_FRAMETYPE_LINK_STATISTICS: {
            memcpy(&CRSF_LinkStatistics, (void *)data + 1, sizeof(CRSF_LinkStatistics));
            break;
        }

        case CRSF_FRAMETYPE_ATTITUDE: {
            if (frame_size != 8) {
                return;
            }

            struct crsfPayloadAttitude_s * crsfAttitude_p = (struct crsfPayloadAttitude_s *)(data + 1);
            CRSF_Attitude.pitch = crsfAttitude_p->pitch;
            CRSF_Attitude.roll = crsfAttitude_p->roll;
            CRSF_Attitude.yaw = crsfAttitude_p->yaw;
            break;
        }
//        default: // undefined frame
    }

    // Write channel data to CSV file
    fprintf(csvFile, "0x%0X,%u,0x%0X,%u,", crsf_packet.device_id, crsf_packet.frame_size, crsf_packet.frame_type, crsf_packet.crc);

    for (int i = 0; i < 16; i++) {
        fprintf(csvFile, "%d,", crsf_packet.received_channels[i]);
    }

    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.uplink_RSSI_1);
    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.uplink_RSSI_2);
    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.uplink_Link_quality);
    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.uplink_SNR);
    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.active_antenna);
    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.rf_Mode);
    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.uplink_TX_Power);
    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.downlink_RSSI);
    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.downlink_Link_quality);
    fprintf(csvFile, "%d,", (int) CRSF_LinkStatistics.downlink_SNR);
    fprintf(csvFile, "%d,", (int) CRSF_Attitude.pitch);
    fprintf(csvFile, "%d,", (int) CRSF_Attitude.roll);
    fprintf(csvFile, "%d,", (int) CRSF_Attitude.yaw);
    fprintf(csvFile, "%d,", (int) crc_failures_count);
    fprintf(csvFile, "%X", (int) dump_position);
    fprintf(csvFile, "\n");
}

int readFromSource(FILE * dumpFile, uint8_t * data, uint16_t buffer_position, uint16_t previously_read, uint8_t n)
{
    size_t bytesRead;

    if (buffer_position + n > BUFFER_SIZE) {
        printf("memcpy\n");
        memcpy((void*)data, (void*) (data + buffer_position), BUFFER_SIZE - buffer_position);
        buffer_position = 0;
        previously_read = BUFFER_SIZE - buffer_position;
    }

    if (buffer_position + n > previously_read) {
        bytesRead = fread(data + previously_read, sizeof(uint8_t), buffer_position + n - previously_read, dumpFile);

        printf("read %d bytes\n", (int)bytesRead);

        if (bytesRead != sizeof(uint8_t) * (buffer_position + n - previously_read)) {
            printf("Error reading the data");
            return -1;
        }

        previously_read = previously_read + bytesRead;
    } else {
        printf("No reads, data is available");
    }

    return previously_read;
}

int main() {
    FILE *dumpFile, *csvFile;
    //char dumpFileName[] = "stm32_dump_20240215_0032_tx_from_fc_telemetry_on_throttle_only.bin";  // Replace with your dump file name
    //char dumpFileName[] = "stm32_dump_20240211_2248_uart_baudrate420_no_moves.bin";
    //char dumpFileName[] = "stm32_dump_20240210_2109_uart_baud420_buffer600clean_throttle_only.bin";
    //char dumpFileName[] = "stm32_dump_20240214_2133_tx_from_fc_disconnect.bin";
    //char dumpFileName[] = "stm32_dump_20240229_225833_tx_from_fc_telemetry_on_roll_pitch_yaw.bin";
    char dumpFileName[] = "stm32_dump_20240229_230424_tx_from_fc_telemetry_on_walking_room.bin";
    char csvFileName[] = "output.csv"; // Replace with your desired CSV file name
    uint8_t device_id;
    uint8_t frame_size;
    uint8_t data[BUFFER_SIZE];
    uint8_t crc;
    uint8_t frameType = 0x16;  // CRSF RC frame type
    uint8_t frame_size_candidate;
    uint16_t buffer_position, previously_read;
    uint32_t dump_position;
    uint32_t crc_failures_count = 0;
    CRSF_LinkStatistics.uplink_RSSI_1 = 0;
    CRSF_LinkStatistics.uplink_RSSI_2 = 0;
    CRSF_LinkStatistics.uplink_Link_quality = 0;
    CRSF_LinkStatistics.uplink_SNR = 0;
    CRSF_LinkStatistics.active_antenna = 0;
    CRSF_LinkStatistics.rf_Mode = 0;
    CRSF_LinkStatistics.uplink_TX_Power = 0;
    CRSF_LinkStatistics.downlink_RSSI = 0;
    CRSF_LinkStatistics.downlink_Link_quality = 0;
    CRSF_LinkStatistics.downlink_SNR = 0;
    CRSF_Attitude.pitch = 0;
    CRSF_Attitude.roll = 0;
    CRSF_Attitude.yaw = 0;

    dumpFile = fopen(dumpFileName, "rb");
    csvFile = fopen(csvFileName, "w");

    if (dumpFile == NULL || csvFile == NULL) {
        perror("Error opening file");
        return 1;
    }

    fprintf(csvFile, "device_id,frame_size,frame_type,crc,in0,in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15,");
    fprintf(csvFile, "uplink_RSSI_1,uplink_RSSI_2,uplink_Link_quality,uplink_SNR,");
    fprintf(csvFile, "active_antenna,rf_Mode,uplink_TX_Power,downlink_RSSI,");
    fprintf(csvFile, "downlink_Link_quality,downlink_SNR,");
    fprintf(csvFile, "pitch,roll,yaw,crc_failures_count,dump_position\n");

    fseek(dumpFile, DUMP_DATA_POSITION, SEEK_SET);
    buffer_position = 0;
    previously_read = 0;
    previously_read = readFromSource(dumpFile, data, buffer_position, previously_read, 1);
    printf("previously_read: %d\n", previously_read);

    // Process the dump file
    while (!feof(dumpFile)) {
        previously_read = readFromSource(dumpFile, data, buffer_position + 1, previously_read, 1);
        device_id = data[buffer_position];
        frame_size = data[buffer_position + 1];
        printf("frame size: %d\n", frame_size);
        previously_read = readFromSource(dumpFile, data, buffer_position + 2, previously_read, frame_size);
        crc = data[buffer_position + frame_size + 1];
        uint8_t crc_fact = calculateCRC(data, buffer_position + 2, frame_size);
        dump_position = ftell(dumpFile);

        if (crc_fact == crc) {
            processCrsfFrame(data + buffer_position + 2, device_id, frame_size, crc, csvFile, crc_failures_count, dump_position);
            //buffer_position = buffer_position + frame_size + 2;
            buffer_position = 0;
            previously_read = 0;
            printf("crc ok, crc = %d, frame_size: %d, dump_position = %d (%X), buffer_position: %d\n", crc, frame_size, dump_position, dump_position, buffer_position);
        } else {
            crc_failures_count++;
            // search for another beginning
            printf("CRC failed: crc fact: %d,  crc read: %d\n", crc_fact, crc);
            printf("frame_size: %d, dump_position = %d (%X), buffer_position: %d\n", frame_size, dump_position, dump_position, buffer_position);
            uint8_t found = 0;

            for(uint16_t i = buffer_position + 1; i < buffer_position + frame_size; i++) {
                dump_position = ftell(dumpFile);
                printf("Searching for new beginning in buffer, i=%d, data[i]=%0X, dump position: %d", i, data[i], dump_position);
                if (data[i] == CRSF_SYNC_BYTE) {
                    buffer_position = i;
                    printf("- Found!");
                    found = 1;
                    break;
                }

                printf("\n");
            }

            if (!found) {
                buffer_position = 0;

                do {
                    readFromSource(dumpFile, data, buffer_position, 0, 1);
                    dump_position = ftell(dumpFile);
                    printf("Searching for new beginning in file, dump position: %X (%d), char: %x", dump_position, dump_position, data[buffer_position]);
                } while(data[buffer_position] != CRSF_SYNC_BYTE && !feof(dumpFile));

                previously_read = 1;
            }
        }
    }

    fclose(dumpFile);
    fclose(csvFile);

    return 0;
}