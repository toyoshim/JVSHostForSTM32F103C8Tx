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
  return HAL_GetTick();
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

  io = JVSIO_open(&data_client, &sense_client, &led_client, &time_client, 0);
  io->begin(io);
}

void JVS_HOST_Run() {
  io->host(io);
}