
typedef struct data_for_flash {
  uint8_t beginning_marker;
  uint32_t tick;
  uint16_t rx_pitch;
  uint16_t rx_roll;
  uint16_t rx_yaw;
  uint16_t rx_trottle;
  uint16_t rx_arm;
  uint pitch;
  uint roll;
  uint yaw;
  uint8_t uplink_quality;
  uint8_t downlink_quality;
  char flight_mode[4];
} data_for_flash_s;
