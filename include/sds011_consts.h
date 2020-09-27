/** @file sds011_consts.h
 * SDS011 constants. These values are protocol specific and should not require
 * change.
 */

#ifndef _SDS011_CONSTS_H
#define _SDS011_CONSTS_H

#include "driver/uart.h"

/*********************************************************
 * UART definitions.
 *********************************************************/

/** UART baud rate. SDS011 uses 9600 bytes. */
#define SDS011_UART_BAUD_RATE 9600
/** UART character size. SDS011 uses 8 bits. */
#define SDS011_UART_DATA_BITS UART_DATA_8_BITS
/** UART parity. SDS011 uses no parity. */
#define SDS011_UART_PARITY UART_PARITY_DISABLE
/** UART stop bits. SDS011 uses one stop bit. */
#define SDS011_UART_STOP_BITS UART_STOP_BITS_1
/** UART flow control. SDS011 does not support flow control. */
#define SDS011_UART_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE
/** UART event queue size. Event queue is disabled, so this value is 0. */
#define SDS011_UART_EVENT_QUEUE_SIZE 0
/** UART flow control threshold. Flow control is disabled, so this value is 0.
 */
#define SDS011_UART_FLOW_CTR_THRESH 0

/*********************************************************
 * Data packet size definitions.
 *********************************************************/

/** TX packet size. */
#define SDS011_PACKET_TX_SIZE 19
/** RX packet size. */
#define SDS011_PACKET_RX_SIZE 10

/** Packet payload size for TX packets. The sub command and the sensor device id
 * bytes are not counted, since they are common across all TX packets.
 */
#define SDS011_PACKET_TX_PAYLOAD_SIZE 12
/** Packet payload size for RX packets. The response device id bytes are not
 * counted, since they are common accross all RX packets. */
#define SDS011_PACKET_RX_PAYLOAD_SIZE 4

/** Since the packet payload fields of this library differ from the actual
 * implementation for practical reasons, the byte ranges for payload checksum
 * calculations need to be defined separately: */

/** Offset of first byte for checksum calculation in RX packets. */
#define SDS011_PACKET_RX_CHECKSUM_OFFSET 2
/** Number of bytes for checksum calculation in RX packets. */
#define SDS011_PACKET_RX_CHECKSUM_BYTES 6

/** Offset of first byte for checksum calculation in TX packets. */
#define SDS011_PACKET_TX_CHECKSUM_OFFSET 2
/** Number of bytes for checksum calculation in TX packets. */
#define SDS011_PACKET_TX_CHECKSUM_BYTES 15

/*********************************************************
 * Constant packet value definitions.
 *********************************************************/

/** Device ID to send command to all devices. */
#define SDS011_ID_ALL 0xFFFF

/** Packet header. Always 0xAA */
#define SDS011_PACKET_HEAD 0xAA
/** Packet tail. Always 0xAB */
#define SDS011_PACKET_TAIL 0xAB

/*********************************************************
 * Command value definitions.
 *********************************************************/

/** Command field value for all TX to the sensor. */
#define SDS011_CMD_TX 0xB4

/** Possible commands the sensor can return. */
typedef uint8_t sds011_rx_cmd;
/** Response to any subcommand changing / querying the sensor config. */
#define SDS011_RX_CMD_CONFIG 0xC5
/** Sensor data from active report or as query response. */
#define SDS011_RX_CMD_SENSOR_DATA 0xC0

/** Available subcommands to send to sensor. */
typedef uint8_t sds011_tx_cmd;
/** Set or query the report mode. Persists power off. */
#define SDS011_TX_CMD_REPORT_MODE 2
/** Query sensor measurement values. **/
#define SDS011_TX_CMD_QUERY_DATA 4
/** Set sensor ID. Persists power off. */
#define SDS011_TX_CMD_SET_ID 5
/** Set or query the sleep mode. Does not persist power off. */
#define SDS011_TX_CMD_SLEEP_MODE 6
/** Query firmware information. */
#define SDS011_TX_CMD_FIRMWARE 7
/** Set or query the working period. Persists power off. */
#define SDS011_TX_CMD_WORKING_PERIOD 8

/** Command method typedef. */
typedef uint8_t sds0110_method;
/** Packet value for getting config data. */
#define SDS011_METHOD_GET 0
/** Packet value for setting getting config data. */
#define SDS011_METHOD_SET 1

/** Report mode typedef.  */
typedef uint8_t sds011_report_mode;
/** Active report mode. Sensor will periodically send data. */
#define SDS011_REPORT_MODE_ACTIVE 0
/** Query report mode. Sensor only send data when active. */
#define SDS011_REPORT_MODE_QUERY 1

/** Sleep mode typedef. */
typedef uint8_t sds011_sleep_mode;
/** Sleep mode. Sensor is in standby. */
#define SDS011_SLEEP_MODE_ENABLED 0
/** Work mode. Sensor is working. */
#define SDS011_SLEEP_MODE_DISABLED 1

/** Broadcast device ID. */
#define SDS011_DEVICE_ID_ALL 0xFFFF

#endif  //_SDS011_CONSTS_H