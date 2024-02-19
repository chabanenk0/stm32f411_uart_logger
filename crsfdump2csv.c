#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define CRSF_SYNC_BYTE 0XC8
#define BUFFER_SIZE 512
#define CSRF_FRAME_RC_REGULAR 0x16
#define DUMP_DATA_POSITION 0x3A01

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

typedef struct {
    uint8_t device_id;
    uint8_t frame_size;
    uint8_t frame_type;
    uint16_t received_channels[16];
    uint16_t telemetry_channels[16];
    uint8_t crc;
} CRSF_Packet;

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
    }

    return crc;
}

void processCrsfFrame(uint8_t * data, uint8_t device_id, uint8_t frame_size, uint8_t crc, FILE * csvFile)
{
    uint8_t frame_type = data[0];
    CRSF_Packet crsf_packet;
    crsf_packet.device_id = device_id;
    crsf_packet.frame_size = frame_size;
    crsf_packet.frame_type = frame_type;
    crsf_packet.crc = crc;

    for(int i = 0; i < 16; i++) {
        crsf_packet.received_channels[i] = 0;
        crsf_packet.telemetry_channels[i] = 0;
    }

    switch(frame_type) {
        case CSRF_FRAME_RC_REGULAR: {
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

//        default: // undefined frame
    }

    // Write channel data to CSV file
    fprintf(csvFile, "0x%0X,%u,0x%0X,%u,", crsf_packet.device_id, crsf_packet.frame_size, crsf_packet.frame_type, crsf_packet.crc);

    for (int i = 0; i < 16; i++) {
        fprintf(csvFile, "%d,", crsf_packet.received_channels[i]);
    }

    fprintf(csvFile, "\n");
}

int readFromSource(FILE * dumpFile, uint8_t * data, uint16_t buffer_position, uint8_t n)
{
    size_t bytesRead;

    if (buffer_position + n > BUFFER_SIZE) {
        printf("memcpy\n");
        memcpy((void*)data, (void*) (data + buffer_position), BUFFER_SIZE - buffer_position);
        buffer_position = 0;
    }

    bytesRead = fread(data + buffer_position, sizeof(uint8_t), n, dumpFile);

    printf("read %d bytes\n", (int)bytesRead);
    if (bytesRead != sizeof(uint8_t) * n) {
        perror("Error reading the data");
        return -1;
    }

    return buffer_position;
}

int main() {
    FILE *dumpFile, *csvFile;
    //char dumpFileName[] = "stm32_dump_20240215_0032_tx_from_fc_telemetry_on_throttle_only.bin";  // Replace with your dump file name
    //char dumpFileName[] = "stm32_dump_20240211_2248_uart_baudrate420_no_moves.bin";
    //char dumpFileName[] = "stm32_dump_20240210_2109_uart_baud420_buffer600clean_throttle_only.bin";
    char dumpFileName[] = "stm32_dump_20240214_2133_tx_from_fc_disconnect.bin";
    char csvFileName[] = "output.csv"; // Replace with your desired CSV file name
    uint8_t device_id;
    uint8_t frame_size;
    uint8_t data[BUFFER_SIZE];
    uint8_t crc;
    uint8_t frameType = 0x16;  // CRSF RC frame type
    uint8_t frame_size_candidate;
    uint16_t buffer_position, buffer_position_new;
    uint32_t dump_position;

    dumpFile = fopen(dumpFileName, "rb");
    csvFile = fopen(csvFileName, "w");

    if (dumpFile == NULL || csvFile == NULL) {
        perror("Error opening file");
        return 1;
    }

    fprintf(csvFile, "device_id,frame_size,frame_type,crc,in0,in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15\n");

    fseek(dumpFile, DUMP_DATA_POSITION, SEEK_SET);
    buffer_position = 0;

    // Process the dump file
    while (!feof(dumpFile)) {
        buffer_position_new = readFromSource(dumpFile, data, buffer_position, 2);
        device_id = data[buffer_position];
        frame_size = data[buffer_position + 1];
        buffer_position += 2;
        buffer_position_new = readFromSource(dumpFile, data, buffer_position_new, frame_size);
        crc = data[buffer_position_new + frame_size - 1];
        uint8_t crc_fact = calculateCRC(data, buffer_position_new, frame_size);
        dump_position = ftell(dumpFile);

        if (crc_fact == crc) {
            processCrsfFrame(data + buffer_position_new, device_id, frame_size, crc, csvFile);
                        printf("crc ok, crc = %d, frame_size: %d, dump_position = %d (%X), buffer_position_new: %d\n", crc, frame_size, dump_position, dump_position, buffer_position_new);
            buffer_position_new = 0;
        } else {
            // search for another beginning
            printf("CRC failed: crc fact: %d,  crc read: %d\n", crc_fact, crc);
            printf("frame_size: %d, dump_position = %d (%X), buffer_position_new: %d\n", frame_size, dump_position, dump_position, buffer_position_new);
            uint8_t found = 0;

            for(uint16_t i = buffer_position_new; i < buffer_position_new + frame_size; i++) {
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
                    readFromSource(dumpFile, data, buffer_position, 1);
                                    dump_position = ftell(dumpFile);
                    printf("Searching for new beginning in file, dump position: %d", dump_position);
                } while(data[buffer_position] != CRSF_SYNC_BYTE && !feof(dumpFile));
            }
        }
    }

    fclose(dumpFile);
    fclose(csvFile);

    return 0;
}