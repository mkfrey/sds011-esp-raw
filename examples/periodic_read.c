/* SDS011-esp-raw usage example.

   Takes measurements periodically and prints them to the serial logging
   console. Sets the sensor to sleep when no measurements are taken.

   This example code assumes that the sensor is in active mode doing continuous
   measurements, which is the default setting of the sensor. The responses to
   the sleep commands are ignored.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sds011.h"
#include "stdio.h"

/** Sensor UART configuration. Adapt to your setup. */
#define SDS011_UART_PORT UART_NUM_2
#define SDS011_RX_GPIO 39
#define SDS011_TX_GPIO 13

/** Time in seconds to let the SDS011 run before taking the measurment. */
#define SDS011_ON_DURATION 15

/** Interval to read and print the sensor values in seconds. Must be bigger than
 * SDS011_ON_DURATION. */
#define PRINT_INTERVAL 60

static const struct sds011_tx_packet sds011_tx_sleep_packet = {
    .head = SDS011_PACKET_HEAD,
    .command = SDS011_CMD_TX,
    .sub_command = SDS011_TX_CMD_SLEEP_MODE,
    .payload_sleep_mode = {.method = SDS011_METHOD_SET,
                           .mode = SDS011_SLEEP_MODE_ENABLED},
    .device_id = SDS011_DEVICE_ID_ALL,
    .tail = SDS011_PACKET_TAIL};

static const struct sds011_tx_packet sds011_tx_wakeup_packet = {
    .head = SDS011_PACKET_HEAD,
    .command = SDS011_CMD_TX,
    .sub_command = SDS011_TX_CMD_SLEEP_MODE,
    .payload_sleep_mode = {.method = SDS011_METHOD_SET,
                           .mode = SDS011_SLEEP_MODE_DISABLED},
    .device_id = SDS011_DEVICE_ID_ALL,
    .tail = SDS011_PACKET_TAIL};

void data_task(void* pvParameters) {
  struct sds011_rx_packet rx_packet;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    /** Wake the sensor up. */
    sds011_send_cmd_to_queue(&sds011_tx_wakeup_packet, 0);

    /** Give it a few seconds to create some airflow. */
    vTaskDelay(pdMS_TO_TICKS(SDS011_ON_DURATION * 1000));

    /** Read the data (which is the latest when data queue size is 1). */
    if (sds011_recv_data_from_queue(&rx_packet, 0) == SDS011_OK) {
      float pm2_5;
      float pm10;

      pm2_5 = ((rx_packet.payload_query_data.pm2_5_high << 8) |
               rx_packet.payload_query_data.pm2_5_low) /
              10.0;
      pm10 = ((rx_packet.payload_query_data.pm10_high << 8) |
              rx_packet.payload_query_data.pm10_low) /
             10.0;

      printf(
          "PM2.5: %.2f\n"
          "PM10: %.2f\n",
          pm2_5, pm10);

      /** Set the sensor to sleep. */
      sds011_send_cmd_to_queue(&sds011_tx_sleep_packet, 0);

      /** Wait for next interval time. */
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PRINT_INTERVAL * 1000));
    }
  }
}

void app_main(void) {
  /** Initialize the SDS011. */
  sds011_begin(SDS011_UART_PORT, SDS011_TX_GPIO, SDS011_RX_GPIO);

  /** Create the measurement task. */
  assert(xTaskCreatePinnedToCore(data_task, "sds011", 2048, NULL, 0, NULL, 1) ==
         pdTRUE);
}
