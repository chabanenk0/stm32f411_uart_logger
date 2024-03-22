#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "Core/Inc/flight_data.h"

#define DUMP_DATA_POSITION 0x7000

FILE * dumpFile;

int data_read(uint8_t * data, int n, int sourceId)
{
    if (1 == sourceId) {
        return fread(data, sizeof(uint8_t), n, dumpFile);
    }

    return -1;
}

void printCsvLine(FILE * csvFile, struct data_for_flash *data_for_flash_p)
{
    fprintf(csvFile, "%d,", (int) data_for_flash_p->tick);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->rx_pitch);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->rx_roll);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->rx_yaw);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->rx_trottle);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->rx_arm);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->pitch);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->roll);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->yaw);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->uplink_quality);
    fprintf(csvFile, "%d,", (int) data_for_flash_p->downlink_quality);
    fprintf(csvFile, "%s", (char *) data_for_flash_p->flight_mode);
    fprintf(csvFile, "\n");
}

int main() {
    FILE *csvFile;
    struct data_for_flash data_for_flash_obj;
    char dumpFileName[] = "stm32_dump_20240322_230317_dump_after_flight_second_success.bin";
    char csvFileName[] = "stm32_dump_20240322_230317_dump_after_flight_second_success.csv";
    dumpFile = fopen(dumpFileName, "rb");
    csvFile = fopen(csvFileName, "w");

    if (dumpFile == NULL || csvFile == NULL) {
        perror("Error opening file");
        return 1;
    }

    fprintf(csvFile, "tick,rx_pitch,rx_roll,rx_yaw,rx_trottle,rx_arm,pitch,roll,yaw,uplink_quality,downlink_quality,flight_mode\n");

    fseek(dumpFile, DUMP_DATA_POSITION, SEEK_SET);

    // Process the dump file
    while (!feof(dumpFile)) {
        fread(&data_for_flash_obj, sizeof(struct data_for_flash), 1, dumpFile);

        if (
            -1 == data_for_flash_obj.tick 
            && 65535 == data_for_flash_obj.rx_pitch
            && 65535 == data_for_flash_obj.rx_roll
            && 65535 == data_for_flash_obj.rx_yaw
            && 65535 == data_for_flash_obj.rx_trottle
        ) {
            break;
        }

        printCsvLine(csvFile, &data_for_flash_obj);
    }

    fclose(dumpFile);
    fclose(csvFile);

    return 0;
}