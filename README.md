# sds011-esp-raw
ESP-IDF component which provides a low level interface to the SDS011 particulate matter
sensor.

## Fundamentals
This library works by using three queues:
1. Outgoing packets to the sensor (commands)
2. Command response packets from the sensor
3. Sensor measurement packets from the sensor

The reason for splitting the packets from the sensor into two queues is to separate explicitly
requested responses from automatically sent sensor measurements when the sensor is in active
mode. In this case the sent measurements might be the first incoming packet after sending a
command, which makes assigning the response packet to the request packet harder.

## Usage
Note: The library can only mange a single UART port with a sensor.

### Initialization
To initialize the library, call
```c
sds011_begin(UART_NUM, TX_IO_NUM, RX_IO_NUM);
```
where `UART_NUM` is the UART port to use, either `UART_NUM_1` or `UART_NUM_2`. `UART_NUM_0`
is also available, but conventionally used for logging output. `TX_IO_NUM` is number of the
GPIO port connected to the `RX` port of the SDS011 and `RX_IO_NUM` the one connected to its
`TX` port.

This function must be called before using any of the functions below.

If the provided parameters are invalid, calling this function will cause an abort.

### Sending packets
Packets can be sent calling
```c
sds011_send_cmd_to_queue(PACKET_PTR, WAIT_TICKS);
```
where `PACKET_PTR` is a pointer to a struct of the type `sds011_tx_packet` and `WAIT_TICKS` is
the number of ticks to wait for an available slot in the queue in case it is currently full.
The packet struct will be copied by the function.

Returns `SDS011_OK` when the packet was successfully enqueued, `SD011_TIMEOUT` otherwise.

An example packet struct can be initialized like this:
```c
struct sds011_tx_packet packet = (struct sds011_tx_packet){
        .head = SDS011_PACKET_HEAD,
        .command = SDS011_CMD_TX,
        .sub_command = SDS011_TX_CMD_SLEEP_MODE,
        .payload_sleep_mode = {.method = SDS011_METHOD_SET,
                               .mode = SDS011_SLEEP_MODE_ENABLED},
        .device_id = SDS011_DEVICE_ID_ALL,
        .tail = SDS011_PACKET_TAIL};
```
All of the values used above are defined in [`sds011_consts.h`][2].

When sending a packet, the `head`, `command` and `tail` fields always contain the same values
as shown above. `device_id` is the ID of the sensor to communicate with, the value of
`SDS011_DEVICE_ID_ALL`, which is `0xFFFF`, acts like a broadcast address.

`sub_command` is one of the `SDS011_TX_CMD_*` values and declares which payload the packet
contains. In this case the sub command specifies that the payload is a sleep mode command.

The `payload_*` fields of the `tx_packet` are an anonymous union, which means only one
of them must be set. In the example above `payload_sleep_mode` is used, since the
`sub_command` field specified the the payload as sleep mode command.

The `checksum` field is automatically populated by the library.

More information about the possible payload structs can be found in the corresponding header
file, [`sds011_structs.h`][1].

### Receiving command response packets
Note: The response to data query commands is a regular sensor data packet, which is sent
to the data queue.

Command response packets can be received calling
```c
sds011_recv_cmd_from_queue(DEST_PACKET_PTR, TICKS_TO_WAIT);
```
where `DEST_PACKET_PTR` is a pointer to a `struct sds011_rx_packet`, to which the received
data is written to and `TICKS_TO_WAIT` is the amount of ticks to wait for an available packet
in case the queue is empty.

Returns `SDS011_OK` when a packet was successfully received, `SD011_TIMEOUT` otherwise.

More information about RX packet structure can also be found in the header file
[`sds011_structs.h`][1].

### Receiving data packets
Command response packets can be received calling
```c
sds011_recv_data_from_queue(DEST_PACKET_PTR, TICKS_TO_WAIT);
```
The usage is the same as receiving command response packets. It can be assumed any packet
received has the payload field `payload_query_data`.

### Deinitialization
The resources used by the library can be released using
```c
sds011_end();
```
The send and receive functions should not be used afterwards if the library does not get
initialized again.

## Examples
See [`periodic_read.c`][3] in the `examples` directory. It shows how to use this library
to send sleep commands and to receive values.

## Resources
This library has been developed using the [Laser Dust Sensor Control Protocol V1.3][4]
documentation from Nova Fitness.

Some findings about the sensor gathered during development and testing can be found in
the [`docs`][5] directory.

[1]: include/sds011_structs.h
[2]: include/sds011_consts.h
[3]: examples/periodic_read.c
[4]: https://nettigo.pl/attachments/415
[5]: docs
