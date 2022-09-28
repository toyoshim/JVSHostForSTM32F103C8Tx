// Copyright 2022 Takashi Toyoshima <toyoshim@gmail.com>. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "jvs_host.h"

#include <stddef.h>
#include <stdio.h>

#include "main.h"  // for GPIO definitions.
#include "stm32f1xx_hal.h"

#include "JVSIO_c.h"

static struct JVSIO_DataClient data_client;
static struct JVSIO_SenseClient sense_client;
static struct JVSIO_LedClient led_client;
static struct JVSIO_TimeClient time_client;
static struct JVSIO_HostClient host_client;
static struct JVSIO_Lib* io = NULL;

extern ADC_HandleTypeDef hadc1;  // for JVS sense
static uint32_t sense_data;

extern UART_HandleTypeDef huart1;  // for debug messages
extern UART_HandleTypeDef huart2;  // for JVS RS-485

static uint8_t tx_buffer[256];
static uint8_t tx_size;
static bool tx_closing;
static uint8_t rx_data;
static bool rx_available;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart != &huart2)
    return;
  tx_size = 0;
  if (tx_closing) {
    HAL_GPIO_WritePin(USART2_DIR_GPIO_Port, USART2_DIR_Pin, GPIO_PIN_RESET);
    tx_closing = false;
  }
}

static int data_available(struct JVSIO_DataClient* client) {
  if (tx_closing)
    return 0;
  if (rx_available)
    return 1;
  if (HAL_OK != HAL_UART_Receive(&huart2, &rx_data, 1, 0))
    return 0;
  rx_available = true;
  return 1;
}

static void data_setInput(struct JVSIO_DataClient* client) {
  tx_closing = true;
  if (tx_size == 0) {
    HAL_GPIO_WritePin(USART2_DIR_GPIO_Port, USART2_DIR_Pin, GPIO_PIN_RESET);
  }
  rx_available = false;
}

static void data_setOutput(struct JVSIO_DataClient* client) {
  HAL_GPIO_WritePin(USART2_DIR_GPIO_Port, USART2_DIR_Pin, GPIO_PIN_SET);
}

static void data_startTransaction(struct JVSIO_DataClient* client) {
  tx_size = 0;
  tx_closing = false;
}

static void data_endTransaction(struct JVSIO_DataClient* client) {
  HAL_UART_Transmit_IT(&huart2, tx_buffer, tx_size);
}

static uint8_t data_read(struct JVSIO_DataClient* client) {
  data_available(client);
  rx_available = false;
  return rx_data;
}

static void data_write(struct JVSIO_DataClient* client, uint8_t data) {
  tx_buffer[tx_size++] = data;
}

static bool data_setCommSupMode(struct JVSIO_DataClient* client,
                                enum JVSIO_CommSupMode mode,
                                bool dryrun) {
  // Not used in the host mode yet.
  return false;
}

// optional for debug logging.
static void data_dump(struct JVSIO_DataClient* client,
                      const char* str,
                      uint8_t* data,
                      uint8_t len) {
  char sbuf[32];
  size_t size = snprintf(sbuf, 32, "%s: ", str);
  HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, size, 1);
  for (uint i = 0; i < len; ++i) {
    size = snprintf(sbuf, 32, "%02x", data[i]);
    HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, size, 1);
  }
  size = snprintf(sbuf, 32, "\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, size, 1);
}

static void sense_begin(struct JVSIO_SenseClient* client) {
  HAL_ADC_Start_DMA(&hadc1, &sense_data, 1);
}

static void sense_set(struct JVSIO_SenseClient* client, bool ready) {
  // Not used in the host mode.
}

static bool sense_isReady(struct JVSIO_SenseClient* client) {
  // See sense_isConnected() below.
  // When all devices are ready, the sense is pulled to GND by the device.
  // So, assumes less than 20000 (~1.0[v]) meant they are ready.
  return sense_data < 20000;
}

static bool sense_isConnected(struct JVSIO_SenseClient* client) {
  // Assumes running at 3.3[V] and ADC pin is 5[V] tolerant.
  // The sense pin should be pulled-up to 3.3+[V].
  // 65535 => 3.3(VDD) - 5.0V : Disconnected
  // The I/O drives the sense signal to 2.5V on their connections.
  // So, assumes less than 60000 meant there is a device on the bus.
  bool connected = sense_data < 60000;

  return connected;
}

// Not used for host.
static void led_begin(struct JVSIO_LedClient* client) {}
static void led_set(struct JVSIO_LedClient* client, bool ready) {}

static void time_delayMicroseconds(struct JVSIO_TimeClient* client,
                                   unsigned int usec) {
  HAL_Delay(usec / 1000);
}

static void time_delay(struct JVSIO_TimeClient* client, unsigned int msec) {
  HAL_Delay(msec);
}

static uint32_t time_getTick(struct JVSIO_TimeClient* client) {
  // Returns msec from the boot.
  return HAL_GetTick();
}

// optional.
static void host_receiveIoId(struct JVSIO_HostClient* client,
                             uint8_t address,
                             uint8_t* data,
                             uint8_t len) {
  char sbuf[256];
  size_t size = snprintf(sbuf, 256, "ID: %s\r\n", data);
  HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, size, 10);
}

// optional.
static void host_receiveCommandRev(struct JVSIO_HostClient* client,
                                   uint8_t address,
                                   uint8_t rev) {
  data_dump(0, "Command Rev", &rev, 1);
}

// optional.
static void host_receiveJvRev(struct JVSIO_HostClient* client,
                              uint8_t address,
                              uint8_t rev) {
  data_dump(0, "Jv Rev", &rev, 1);
}

// optional.
static void host_receiveProtocolVer(struct JVSIO_HostClient* client,
                                    uint8_t address,
                                    uint8_t ver) {
  data_dump(0, "Protocol Ver", &ver, 1);
}

// optional.
static void host_receiveFunctionCheck(struct JVSIO_HostClient* client,
                                      uint8_t address,
                                      uint8_t* data,
                                      uint8_t len) {
  data_dump(0, "Function Check", data, len);
}

// Receives up-to-date switch states after the `sync` call.
// Multiple JVS I/O devices up to 4 can be merged to represent 4 players.
//   players: 1-4
//   coin_state: bitN - player(N+1) coin ON(1)/OFF(0)
//               bit7 - test switch ON(1)/OFF(0)
//   sw_state0[N]: player(N+1)
//      bit7: start
//      bit6: service
//      bit5: up
//      bit4: down
//      bit3: left
//      bit2: right
//      bit1: push button 1
//      bit0: push button 2
//   sw_state1[N]: player(N+1)
//      bit7: push button 3
//      bit6: push button 4
//      bit5: push button 5
//      bit4: push button 6
//      bit3: push button 7
//      bit2: push button 8
//      bit1: push button 9
//      bit0: push button 10
static void host_synced(struct JVSIO_HostClient* client,
                        uint8_t players,
                        uint8_t coin_state,
                        uint8_t* sw_state0,
                        uint8_t* sw_state1) {
  // Example to dump all information.
  char sbuf[256];
  size_t size = snprintf(sbuf, 256, "%s\r\n", "--------------------");
  HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, size, 10);
  size = snprintf(sbuf, 256, "Players: %d\r\n", players);
  HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, size, 10);
  size =
      snprintf(sbuf, 256, "Test: %s\r\n", (coin_state & 0x80) ? "ON" : "OFF");
  HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, size, 10);
  for (uint8_t player = 0; player < players; ++player) {
    size = snprintf(sbuf, 256, "Coin %d: %s\r\n", player + 1,
                    (coin_state & (1 << player)) ? "ON" : "OFF");
    HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, size, 10);
  }
  size = snprintf(sbuf, 256, "%s\r\n", "  StSvUpDnLtRtP1P2P3P4P5P6P7P8P9Pa");
  HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, size, 10);
  size = snprintf(sbuf, 256, "%s\r\n", "P0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0");
  for (uint8_t player = 0; player < players; ++player) {
    sbuf[1] = '1' + player;
    sbuf[3] = (sw_state0[player] & 0x80) ? '1' : '0';
    sbuf[5] = (sw_state0[player] & 0x40) ? '1' : '0';
    sbuf[7] = (sw_state0[player] & 0x20) ? '1' : '0';
    sbuf[9] = (sw_state0[player] & 0x10) ? '1' : '0';
    sbuf[11] = (sw_state0[player] & 0x08) ? '1' : '0';
    sbuf[13] = (sw_state0[player] & 0x04) ? '1' : '0';
    sbuf[15] = (sw_state0[player] & 0x02) ? '1' : '0';
    sbuf[17] = (sw_state0[player] & 0x01) ? '1' : '0';
    sbuf[19] = (sw_state1[player] & 0x80) ? '1' : '0';
    sbuf[21] = (sw_state1[player] & 0x40) ? '1' : '0';
    sbuf[23] = (sw_state1[player] & 0x20) ? '1' : '0';
    sbuf[25] = (sw_state1[player] & 0x10) ? '1' : '0';
    sbuf[27] = (sw_state1[player] & 0x08) ? '1' : '0';
    sbuf[29] = (sw_state1[player] & 0x04) ? '1' : '0';
    sbuf[31] = (sw_state1[player] & 0x02) ? '1' : '0';
    sbuf[33] = (sw_state1[player] & 0x01) ? '1' : '0';
    HAL_UART_Transmit(&huart1, (uint8_t*)sbuf, 36, 10);
  };
}

void JVS_HOST_Init() {
  data_client.available = data_available;
  data_client.setInput = data_setInput;
  data_client.setOutput = data_setOutput;
  data_client.startTransaction = data_startTransaction;
  data_client.endTransaction = data_endTransaction;
  data_client.read = data_read;
  data_client.write = data_write;
  data_client.setCommSupMode = data_setCommSupMode;
  data_client.dump = data_dump;

  sense_client.begin = sense_begin;
  sense_client.set = sense_set;
  sense_client.isReady = sense_isReady;
  sense_client.isConnected = sense_isConnected;

  led_client.begin = led_begin;
  led_client.set = led_set;

  time_client.delayMicroseconds = time_delayMicroseconds;
  time_client.delay = time_delay;
  time_client.getTick = time_getTick;

  host_client.receiveIoId = host_receiveIoId;                    // or 0
  host_client.receiveCommandRev = host_receiveCommandRev;        // or 0
  host_client.receiveJvRev = host_receiveJvRev;                  // or 0
  host_client.receiveProtocolVer = host_receiveProtocolVer;      // or 0
  host_client.receiveFunctionCheck = host_receiveFunctionCheck;  // or 0
  host_client.synced = host_synced;

  io = JVSIO_open(&data_client, &sense_client, &led_client, &time_client, 0);
  io->begin(io);
}

void JVS_HOST_Run() {
  if (io->host(io, &host_client)) {
    // Call `sync` while the bus is ready, or once per frame.
    // `synced` will be called when the response is parsed.
    io->sync(io);
  }
}