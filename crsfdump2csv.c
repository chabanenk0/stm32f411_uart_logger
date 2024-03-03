#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "crsf.h"

#define DUMP_DATA_POSITION 0x3A00


CRSF_Packet crsf_packet;
struct crsfPayloadAttitude_s CRSF_Attitude;
struct crsfPayloadLinkstatistics_s CRSF_LinkStatistics;

// idea source https://github.com/betaflight/betaflight/blob/4.0-maintenance/src/main/common/crc.c#L62

FILE * dumpFile;

int data_read(uint8_t * data, int n, int sourceId)
{
    if (1 == sourceId) {
        return fread(data, sizeof(uint8_t), n, dumpFile);
    }

    return -1;
}


int main() {
    FILE *csvFile;
    //char dumpFileName[] = "stm32_dump_20240215_0032_tx_from_fc_telemetry_on_throttle_only.bin";  // Replace with your dump file name
    //char dumpFileName[] = "stm32_dump_20240211_2248_uart_baudrate420_no_moves.bin";
    //char dumpFileName[] = "stm32_dump_20240210_2109_uart_baud420_buffer600clean_throttle_only.bin";
    //char dumpFileName[] = "stm32_dump_20240214_2133_tx_from_fc_disconnect.bin";
    char dumpFileName[] = "stm32_dump_20240229_225833_tx_from_fc_telemetry_on_roll_pitch_yaw.bin";
    //char dumpFileName[] = "stm32_dump_20240229_230424_tx_from_fc_telemetry_on_walking_room.bin";
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
    clear_link_statistics(&CRSF_LinkStatistics);
    clear_attitude(&CRSF_Attitude);

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
    previously_read = readFromSource(1, data, buffer_position, previously_read, 1);
    printf("previously_read: %d\n", previously_read);

    // Process the dump file
    while (!feof(dumpFile)) {
        previously_read = readFromSource(1, data, buffer_position + 1, previously_read, 1);
        device_id = data[buffer_position];
        frame_size = data[buffer_position + 1];
        printf("frame size: %d\n", frame_size);
        previously_read = readFromSource(1, data, buffer_position + 2, previously_read, frame_size);
        crc = data[buffer_position + frame_size + 1];
        uint8_t crc_fact = calculateCRC(data, buffer_position + 2, frame_size);
        dump_position = ftell(dumpFile);

        if (crc_fact == crc) {
            processCrsfFrame(data + buffer_position + 2, device_id, frame_size, crc, csvFile, crc_failures_count, dump_position, &crsf_packet, &CRSF_LinkStatistics, &CRSF_Attitude);
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
                    readFromSource(1, data, buffer_position, 0, 1);
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