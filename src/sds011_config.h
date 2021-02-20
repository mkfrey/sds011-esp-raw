/** @file sds011_config.h
 * SDS011 configuration file. Allows setting default parameters and some
 * internal values.
 */

/** Outgoing (TX) packet queue size. */
#define SDS011_TX_QUEUE_SIZE 5
/** Incoming (RX) command response queue size. If this is set to 1, queue data
 * is overriden by new data when the queue is full. If bigger than 1, new data
 * is discarded in this case. */
#define SDS011_RX_CMD_QUEUE_SIZE 1
/** Incoming (RX) sensor data queue size. If this is set to 1, queue data is
 * overriden by new data when the queue is full. If bigger than 1, new data is
 * discarded in this case. */
#define SDS011_RX_DATA_QUEUE_SIZE 1

/** TX buffer size (multiples of 1024).  */
#define SDS011_UART_TX_BUFFER_SIZE (1024 * 1)
/** RX buffer size (multiples of 1024). */
#define SDS011_UART_RX_BUFFER_SIZE (1024 * 1)

/** Name of the task processing the UART data. */
#define SDS011_TX_TASK_NAME "sds011tx"
#define SDS011_RX_TASK_NAME "sds011rx"
#define SDS011_TX_TASK_STACK_DEPTH 2048
#define SDS011_RX_TASK_STACK_DEPTH 2048

/** Enable debugging output for development purposes. */
//#define SDS011_DEBUG
