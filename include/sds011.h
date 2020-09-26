/** @file sds011.h
 * SDS011 raw sensor library for the ESP-IDF.
 */

#ifndef _SDS011_H
#define _SDS011_H

#include "sds011_structs.h"

#include <esp_types.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

typedef enum {
  /* No error occured. */
  SDS011_OK = 0,

  /* Another operation is already running. */
  SDS011_ERR_OPERATION_IN_PROGRESS,

  /* Operation timed out. */
  SDS011_ERR_TIMEOUT
} sds011_err;

/**
 * Allocate ressources and start the processing tasks.
 * Must be called once before using the queue operations.
 * @note Will abort when an error occurs.
 * @param uart_num UART port number to use.
 * @param tx_io_num GPIO number to use for UART TX.
 * @param rx_io_num GPIO number to use for UART RX.
 */
void sds011_begin(const uart_port_t uart_num,
                  const int tx_io_num,
                  const int rx_io_num);

/**
 * Free the ressources and stop the processing tasks.
 */
void sds011_end();

/**
 * Send a raw command packet to the TX queue. Checksum field of the package will
 * be properly populated.
 * @param data Pointer to a TX packet structure. Will be copied.
 * @param ticks_to_wait Ticks to wait for space in the queue to become
 * available, if the queue is full.
 * @return `SDS011_OK` if the operation was successful, one of `SDS011_ERR_*`
 * otherwise.
 */
sds011_err sds011_send_cmd_to_queue(const struct sds011_tx_packet* data,
                                    TickType_t ticks_to_wait);

/**
 * Receive a raw packet from the RX command response queue.
 * @param data Pointer to a RX packet structure where the data is written to.
 * @param ticks_to_wait Ticks to wait for a packet to become available in the
 * queue, if the queue is empty.
 * @return `SDS011_OK` if the operation was successful, one of `SDS011_ERR_*`
 * otherwise.
 */
sds011_err sds011_recv_cmd_from_queue(struct sds011_rx_packet* data_out,
                                      TickType_t ticks_to_wait);

/**
 * Receive a raw packet from the RX sensor data queue.
 * @param data Pointer to a RX packet structure where the data is written to.
 * @param ticks_to_wait Ticks to wait for a packet to become available in the
 * queue, if the queue is empty.
 * @return `SDS011_OK` if the operation was successful, one of `SDS011_ERR_*`
 * otherwise.
 */
sds011_err sds011_recv_data_from_queue(struct sds011_rx_packet* data_out,
                                       TickType_t ticks_to_wait);

#endif  //_SDS011_H