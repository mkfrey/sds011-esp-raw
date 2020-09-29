#include "sds011.h"
#include "assert.h"
#include "sds011_config.h"
#include "sds011_consts.h"
#include "sds011_structs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef SDS011_DEBUG
#include "stdio.h"
#endif

/** ESP-IDF v3 compatibility. */
#ifndef GPIO_NUM_NC
#define GPIO_NUM_NC -1
#endif

/** UART num for RX/TX. */
uart_port_t sds011_uart_num;

/** Queue for packages to send. */
QueueHandle_t sds011_tx_queue = NULL;
/** Queue for received command response packages. */
QueueHandle_t sds011_rx_cmd_queue = NULL;
/** Queue for received data packages. */
QueueHandle_t sds011_rx_data_queue = NULL;

/** Task handler for TX task. */
TaskHandle_t sds011_tx_task_handle = NULL;
/** Task handler for RX task. */
TaskHandle_t sds011_rx_task_handle = NULL;

/** TX task function. */
void sds011_tx_task(void* pvParameters);

/** RX task function. */
void sds011_rx_task(void* pvParameters);

/** Verifies head and tail of an RX packet. */
bool sds011_rx_check_frame(const struct sds011_rx_packet* packet);

/** Verifies the checkxum of an RX packet. */
bool sds011_rx_check_checksum(const struct sds011_rx_packet* packet);

/** Calculates the checksum of a TX packet and puts it in its checksum field. */
void sds011_tx_fill_checksum(struct sds011_tx_packet* packet);

void sds011_begin(const uart_port_t uart_num,
                  const int tx_io_num,
                  const int rx_io_num) {
  /** Initialize the data queues. */
  sds011_tx_queue =
      xQueueCreate(SDS011_TX_QUEUE_SIZE, sizeof(struct sds011_tx_packet));
  assert(sds011_tx_queue);
  sds011_rx_cmd_queue =
      xQueueCreate(SDS011_RX_CMD_QUEUE_SIZE, sizeof(struct sds011_rx_packet));
  assert(sds011_rx_cmd_queue);
  sds011_rx_data_queue =
      xQueueCreate(SDS011_RX_DATA_QUEUE_SIZE, sizeof(struct sds011_rx_packet));
  assert(sds011_rx_data_queue);

  /** Set up UART. */
  sds011_uart_num = uart_num;

  uart_config_t uart_config = {
      .baud_rate = SDS011_UART_BAUD_RATE,
      .data_bits = SDS011_UART_DATA_BITS,
      .parity = SDS011_UART_PARITY,
      .stop_bits = SDS011_UART_STOP_BITS,
      .flow_ctrl = SDS011_UART_FLOW_CTRL,
      .rx_flow_ctrl_thresh = SDS011_UART_FLOW_CTR_THRESH
#ifdef CONFIG_PM_ENABLE
      ,
#ifdef UART_SCLK_REF_TICK
      .source_clk = UART_SCLK_REF_TICK
#else
      .use_ref_tick = true
#endif
#endif
  };

  ESP_ERROR_CHECK(uart_param_config(sds011_uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(sds011_uart_num, tx_io_num, rx_io_num,
                               GPIO_NUM_NC, GPIO_NUM_NC));
  ESP_ERROR_CHECK(uart_driver_install(
      sds011_uart_num, SDS011_UART_RX_BUFFER_SIZE, SDS011_UART_TX_BUFFER_SIZE,
      SDS011_UART_EVENT_QUEUE_SIZE, NULL, 0));

  /** Create the TX and RX tasks. */
  assert(xTaskCreatePinnedToCore(sds011_tx_task, SDS011_TX_TASK_NAME,
                                 SDS011_TX_TASK_STACK_DEPTH, NULL, 1,
                                 &sds011_tx_task_handle, 1) == pdPASS);

  assert(xTaskCreatePinnedToCore(sds011_rx_task, SDS011_RX_TASK_NAME,
                                 SDS011_RX_TASK_STACK_DEPTH, NULL, 1,
                                 &sds011_rx_task_handle, 1) == pdPASS);
}

void sds011_end() {
  /** Delete the TX and RX tasks.*/
  vTaskDelete(sds011_tx_task_handle);
  vTaskDelete(sds011_rx_task_handle);

  /** Delete the UART driver. */
  ESP_ERROR_CHECK(uart_driver_delete(sds011_uart_num));

  /** Delete the queues. */
  vQueueDelete(sds011_tx_queue);
  vQueueDelete(sds011_rx_cmd_queue);
  vQueueDelete(sds011_rx_data_queue);
}

/** SDS011 TX task. Writes packets from the TX queue to UART. */
void sds011_tx_task(void* pvParameters) {
  struct sds011_tx_packet packet_send;
  uint8_t* packet_send_buf = (uint8_t*)(&packet_send);
  uint packet_send_remaining = 0;

  for (;;) {
    if (packet_send_remaining == 0) {
      if (xQueueReceive(sds011_tx_queue, (void*)packet_send_buf,
                        portMAX_DELAY) == pdTRUE) {
        sds011_tx_fill_checksum(&packet_send);
        packet_send_remaining = sizeof(packet_send);
      }
    } else {
      int send_bytes = uart_write_bytes(
          sds011_uart_num,
          (char*)&(
              packet_send_buf[sizeof(packet_send) - packet_send_remaining]),
          packet_send_remaining);
      if (send_bytes > 0) {
        packet_send_remaining -= send_bytes;
      }
#ifdef SDS011_DEBUG
      if (packet_send_remaining == 0) {
        printf(
            "---------------\n"
            "New outgoing packet. Dump:\n\n");
        for (int i = 0; i < sizeof(packet_send); ++i) {
          printf("0x%.2X\n", packet_send_buf[i]);
        }
        printf("---------------\n");
      }
#endif
    }
    vTaskDelay(1);
  }
}

/** The RX task is a separate task so it can block indefinitely while no data is
 * received from UART. */
void sds011_rx_task(void* pvParameters) {
  struct sds011_rx_packet packet_recv;
  uint8_t* packet_recv_buf = (uint8_t*)(&packet_recv);
  uint packet_recv_remaining = sizeof(packet_recv);

  for (;;) {
    /** Read RX queue data from the bus. */
    int recv_bytes = uart_read_bytes(
        sds011_uart_num,
        &packet_recv_buf[sizeof(packet_recv) - packet_recv_remaining],
        packet_recv_remaining, portMAX_DELAY);
    if (recv_bytes > 0) {
      packet_recv_remaining -= recv_bytes;
    }

    if (!packet_recv_remaining) {
#ifdef SDS011_DEBUG
      printf(
          "---------------\n"
          "New incoming packet. Dump:\n\n");
      for (int i = 0; i < sizeof(packet_recv); ++i) {
        printf("0x%.2X\n", packet_recv_buf[i]);
      }
      printf("---------------\n");
#endif

      if (sds011_rx_check_frame(&packet_recv) &&
          sds011_rx_check_checksum(&packet_recv)) {
        switch (packet_recv.command) {
          case SDS011_RX_CMD_SENSOR_DATA:
#if SDS011_RX_DATA_QUEUE_SIZE == 1
            assert(xQueueOverwrite(sds011_rx_data_queue,
                                   (const void*)&packet_recv) == pdTRUE);
#else
            xQueueSendToBack(sds011_data_queue, (const void*)&data,
                             (TickType_t)0);
#endif
            break;
          default:
#if SDS011_RX_CMD_QUEUE_SIZE == 1
            assert(xQueueOverwrite(sds011_rx_cmd_queue,
                                   (const void*)&packet_recv) == pdTRUE);
#else
            xQueueSendToBack(sds011_rx_cmd_queue, (const void*)&data,
                             (TickType_t)0);
#endif
            break;
        }
      } else {
        /* Flush UART input in an error case. */
        uart_flush(sds011_uart_num);

#ifdef SDS011_DEBUG
        printf("Received invalid packet!\n");
#endif
      }

      packet_recv_remaining = sizeof(packet_recv);
    }

    vTaskDelay(1);
  }
}

bool sds011_rx_check_frame(const struct sds011_rx_packet* packet) {
  if (packet->head != SDS011_PACKET_HEAD ||
      packet->tail != SDS011_PACKET_TAIL) {
    return false;
  }
  return true;
}

bool sds011_rx_check_checksum(const struct sds011_rx_packet* packet) {
  uint checksum = 0;
  const uint8_t* bytes =
      &(((const uint8_t*)(packet))[SDS011_PACKET_RX_CHECKSUM_OFFSET]);

  for (int i = 0; i < SDS011_PACKET_RX_CHECKSUM_BYTES; ++i) {
    checksum += bytes[i];
  }
  return ((checksum & 0xFF) == packet->checksum);
}

void sds011_tx_fill_checksum(struct sds011_tx_packet* packet) {
  uint checksum = 0;
  const uint8_t* bytes =
      &(((const uint8_t*)(packet))[SDS011_PACKET_TX_CHECKSUM_OFFSET]);

  for (int i = 0; i < SDS011_PACKET_TX_CHECKSUM_BYTES; ++i) {
    checksum += bytes[i];
  }
  packet->checksum = (checksum & 0xFF);
}

sds011_err sds011_recv_data_from_queue(struct sds011_rx_packet* data_out,
                                       TickType_t ticks_to_wait) {
  if (xQueueReceive(sds011_rx_data_queue, (void*)data_out, ticks_to_wait) ==
      pdTRUE)
    return SDS011_OK;
  return SDS011_ERR_TIMEOUT;
}

sds011_err sds011_recv_cmd_from_queue(struct sds011_rx_packet* data_out,
                                      TickType_t ticks_to_wait) {
  if (xQueueReceive(sds011_rx_cmd_queue, (void*)data_out, ticks_to_wait) ==
      pdTRUE) {
    return SDS011_OK;
  }

  return SDS011_ERR_TIMEOUT;
}

sds011_err sds011_send_cmd_to_queue(const struct sds011_tx_packet* data,
                                    TickType_t ticks_to_wait) {
  if (xQueueSend(sds011_tx_queue, (void*)data, ticks_to_wait) == pdTRUE) {
    return SDS011_OK;
  }

  return SDS011_ERR_TIMEOUT;
}
