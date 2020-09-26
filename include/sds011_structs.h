#ifndef _SDS011_STRUCTS_H
#define _SDS011_STRUCTS_H

#include <esp_types.h>
#include "sds011_consts.h"

/*********************************************************
 * TX payload structures.
 *********************************************************/

/** Reporting mode query/set tx packet payload. */
struct sds011_tx_payload_reporting_mode {
  /** Specifies whether the report mode shoud be set or read. */
  sds0110_method method;
  /** Specifies whether the active or query mode should be set. */
  sds011_report_mode mode;
  /** Unused (reserved) bytes. Should be filled with zeros. */
  uint8_t reserved[10];
} __attribute((packed));
_Static_assert(SDS011_PACKET_TX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_tx_payload_reporting_mode),
               "Struct size needs to be the same as payload size.");

/** Data query command payload. */
struct sds011_tx_payload_query_data {
  /** Unused (reserved) bytes. Should be filled with zeros. */
  uint8_t reserved[12];
};
_Static_assert(SDS011_PACKET_TX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_tx_payload_query_data),
               "Struct size needs to be the same as payload size.");

/** Device ID set tx packet payload. */
struct sds011_tx_payload_set_device_id {
  uint8_t reserved[10];
  uint16_t new_device_id;
} __attribute((packed));
_Static_assert(SDS011_PACKET_TX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_tx_payload_set_device_id),
               "Struct size needs to be the same as payload size.");

/** Sleep mode query/set tx packet payload. */
struct sds011_tx_payload_sleep_mode {
  /** Specifies whether the sleep mode shoud be set or read. */
  sds0110_method method;
  /** Specifies whether the sleep or work mode should be set. */
  sds011_sleep_mode mode;
  /** Unused (reserved) bytes. Should be filled with zeros. */
  uint8_t reserved[10];
} __attribute((packed));
_Static_assert(SDS011_PACKET_TX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_tx_payload_sleep_mode),
               "Struct size needs to be the same as payload size.");

/** Working period query/set tx packet payload. */
struct sds011_tx_payload_working_period {
  /** Specifies whether the working period shoud be set or read. */
  sds0110_method method;
  /** The working period to set. A value from 0 (continuous) to 30 (30 minutes).
   * The sensor will work 30 seconds and then sleep (n * 60 - 30) seconds. */
  uint8_t period;
  /** Unused (reserved) bytes. Should be filled with zeros. */
  uint8_t reserved[10];
} __attribute((packed));
_Static_assert(SDS011_PACKET_TX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_tx_payload_working_period),
               "Struct size needs to be the same as payload size.");

/** Firmware version query tx packet payload. */
struct sds011_tx_payload_check_firmware {
  /** Unused (reserved) bytes. Should be filled with zeros. */
  uint8_t reserved[12];
} __attribute((packed));
_Static_assert(SDS011_PACKET_TX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_tx_payload_check_firmware),
               "Struct size needs to be the same as payload size.");

/*********************************************************
 * RX response payload structures.
 *********************************************************/

/** Reporting mode query/set response rx packet payload. */
struct sds011_rx_payload_reporting_mode {
  /** Mirror of the sent sub command. */
  sds011_tx_cmd sub_command;
  /** Mirror of the sent method. */
  sds0110_method method;
  /** Mirror of the sent mode. */
  sds011_report_mode mode;
  /** Unused (reserved) bytes. */
  uint8_t reserved[1];
} __attribute((packed));
_Static_assert(SDS011_PACKET_RX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_rx_payload_reporting_mode),
               "Struct size needs to be the same as payload size.");

/** Data query response rx packet payload. */
struct sds011_rx_payload_query_data {
  /** Low byte of the PM2.5 value. */
  uint8_t pm2_5_low;
  /** High byte of the PM2.5 value. */
  uint8_t pm2_5_high;
  /** Low byte of the PM10 value. */
  uint8_t pm10_low;
  /** High byte of the PM10 value. */
  uint8_t pm10_high;
};
_Static_assert(SDS011_PACKET_RX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_rx_payload_query_data),
               "Struct size needs to be the same as payload size.");

/** Device ID set response rx packet payload. */
struct sds011_rx_payload_set_device_id {
  /** Mirror of the sent sub command. */
  sds011_tx_cmd sub_command;
  uint8_t reserved[3];
} __attribute((packed));
_Static_assert(SDS011_PACKET_RX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_rx_payload_set_device_id),
               "Struct size needs to be the same as payload size.");

/** Sleep mode query/set response rx packet payload. */
struct sds011_rx_payload_sleep_mode {
  /** Mirror of the sent sub command. */
  sds011_tx_cmd sub_command;
  /** Mirror of the sent method. */
  sds0110_method method;
  /** Mirror of the sent mode. */
  sds011_sleep_mode mode;
  /** Unused (reserved) bytes. */
  uint8_t reserved[1];
} __attribute((packed));
_Static_assert(SDS011_PACKET_RX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_rx_payload_sleep_mode),
               "Struct size needs to be the same as payload size.");

/** Working period query/set response rx payload. */
struct sds011_rx_payload_working_period {
  /** Mirror of the sent sub command. */
  sds011_tx_cmd sub_command;
  /** Mirror of the sent method. */
  sds0110_method method;
  /** Mirror of the sent period. */
  uint8_t period;
  /** Unused (reserved) bytes. */
  uint8_t reserved[1];
} __attribute((packed));
_Static_assert(SDS011_PACKET_RX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_rx_payload_working_period),
               "Struct size needs to be the same as payload size.");

/** Firmware version query response rx packet payload. */
struct sds011_rx_payload_check_firmware {
  /** Mirror of the sent sub command. */
  sds011_tx_cmd sub_command;
  /** Firmware year */
  uint8_t year;
  /** Firmware month. */
  uint8_t month;
  /** Firmware day. */
  uint8_t day;
} __attribute((packed));
_Static_assert(SDS011_PACKET_RX_PAYLOAD_SIZE ==
                   sizeof(struct sds011_rx_payload_check_firmware),
               "Struct size needs to be the same as payload size.");

/*********************************************************
 * Generic TX and RX data structures.
 *********************************************************/

/** Generic TX packet structure. */
struct sds011_tx_packet {
  /** Head value. Always 0xAA. */
  uint8_t head;
  /** Command. Always 0xB4. */
  uint8_t command;
  /** Sub command. This is part of the payload, but since all TX packets use
  this byte for the same purpose, it is part of this generic structure. */
  sds011_tx_cmd sub_command;
  /** Packet payloads. */
  union {
    struct sds011_tx_payload_reporting_mode payload_report_mode;
    struct sds011_tx_payload_query_data payload_query_data;
    struct sds011_tx_payload_set_device_id payload_set_device_id;
    struct sds011_tx_payload_sleep_mode payload_sleep_mode;
    struct sds011_tx_payload_working_period payload_working_period;
    struct sds011_tx_payload_check_firmware payload_check_firmware;
  };
  /** Destination sensor device id. This is part of the payload, but
  since all TX packets use this field for the same purpose, it is part of this
  generic structure. */
  uint16_t device_id;
  /** Checksum. */
  uint8_t checksum;
  /** Packet tail. Always 0xAB. */
  uint8_t tail;
} __attribute((packed));
_Static_assert(SDS011_PACKET_TX_SIZE == sizeof(struct sds011_tx_packet),
               "Struct size needs to be the same as payload size.");

/** Generic RX packet structure. */
struct sds011_rx_packet {
  /** Head value. */
  uint8_t head;
  /** Command. */
  uint8_t command;
  /** Packet payloads. */
  union {
    /** Direct access to the mirrored sub command inside the payload, which is
     * present on all payloads except sensor data. */
    sds011_tx_cmd sub_command;
    struct sds011_rx_payload_reporting_mode payload_report_mode;
    struct sds011_rx_payload_query_data payload_query_data;
    struct sds011_rx_payload_set_device_id payload_set_device_id;
    struct sds011_rx_payload_sleep_mode payload_sleep_mode;
    struct sds011_rx_payload_working_period payload_working_period;
    struct sds011_rx_payload_check_firmware payload_check_firmware;
  };
  /** Source sensor device id. This is part of the payload, but
  since all RX packets use this field for the same purpose, it is part of this
  generic structure. Has a special role when setting device id, in this case
  this field contains the new device id. */
  uint16_t device_id;
  /** Checksum. */
  uint8_t checksum;
  /** Packet tail. */
  uint8_t tail;
} __attribute((packed));
_Static_assert(SDS011_PACKET_RX_SIZE == sizeof(struct sds011_rx_packet),
               "Struct size needs to be the same as payload size.");

#endif  //_SDS011_STRUCTS_H