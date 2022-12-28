// Copyright 2022 Takashi Toyoshima <toyoshim@gmail.com>. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "jvs_host.h"

#include <stddef.h>
#include <stdio.h>

#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"
#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/usart.h"

#include "JVSIO_c.h"

#define _DBGLOG

static struct JVSIO_DataClient data_client;
static struct JVSIO_SenseClient sense_client;
static struct JVSIO_LedClient led_client;
static struct JVSIO_TimeClient time_client;
static struct JVSIO_HostClient host_client;
static struct JVSIO_Lib* io = NULL;

static volatile uint32_t tick = 0;

static uint32_t sense_data;

static volatile uint8_t tx_buffer[256];
static volatile uint8_t tx_index;
static volatile uint8_t tx_size;
static volatile bool tx_closing;
static uint8_t rx_data;
static bool rx_available;

void sys_tick_handler(void) {
  tick++;
}

static void usart1_tx_init(void) {
#if defined(_DBGLOG)
  // Activate UART1 TX for debug logging.
  rcc_periph_clock_enable(RCC_USART1);

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART1_TX);

  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  usart_enable(USART1);
#endif
}

static void usart1_tx_write(char* data, size_t size) {
#if defined(_DBGLOG)
  for (size_t i = 0; i < size; ++i)
    usart_send_blocking(USART1, data[i]);
#else
  (void)data;
  (void)size;
#endif
}

void usart2_isr(void) {
  if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
      (USART_SR(USART2) & USART_SR_TXE) != 0) {
    usart_send(USART2, tx_buffer[tx_index++]);
    if (tx_index == tx_size) {
      usart_disable_tx_interrupt(USART2);
      usart_enable_tx_complete_interrupt(USART2);
    }
  }
  if (((USART_CR1(USART2) & USART_CR1_TCIE) != 0) &&
      (USART_SR(USART2) & USART_SR_TC) != 0) {
    usart_disable_tx_complete_interrupt(USART2);
    tx_size = 0;
    if (tx_closing) {
      gpio_clear(GPIOA, GPIO1);
      tx_closing = false;
    }
  }
}

static int data_available(struct JVSIO_DataClient* client) {
  (void)client;
  if (tx_closing)
    return 0;
  if (rx_available)
    return 1;

  if (!usart_get_flag(USART2, USART_SR_RXNE))
    return 0;
  rx_data = usart_recv(USART2);
  rx_available = true;
  return 1;
}

static void data_setInput(struct JVSIO_DataClient* client) {
  (void)client;
  tx_closing = true;
  if (tx_size == 0) {
    gpio_clear(GPIOA, GPIO1);
  }
  rx_available = false;
}

static void data_setOutput(struct JVSIO_DataClient* client) {
  (void)client;
  gpio_set(GPIOA, GPIO1);
}

static void data_startTransaction(struct JVSIO_DataClient* client) {
  (void)client;
  tx_size = 0;
  tx_index = 0;
  tx_closing = false;
}

static void data_endTransaction(struct JVSIO_DataClient* client) {
  (void)client;
  usart_enable_tx_interrupt(USART2);
}

static uint8_t data_read(struct JVSIO_DataClient* client) {
  data_available(client);
  rx_available = false;
  return rx_data;
}

static void data_write(struct JVSIO_DataClient* client, uint8_t data) {
  (void)client;
  tx_buffer[tx_size++] = data;
}

static bool data_setCommSupMode(struct JVSIO_DataClient* client,
                                enum JVSIO_CommSupMode mode,
                                bool dryrun) {
  (void)client;
  (void)mode;
  (void)dryrun;
  // Not used in the host mode yet.
  return false;
}

// optional for debug logging.
static void data_dump(struct JVSIO_DataClient* client,
                      const char* str,
                      uint8_t* data,
                      uint8_t len) {
  (void)client;
  char sbuf[32];
  size_t size = snprintf(sbuf, 32, "%s: ", str);
  usart1_tx_write(sbuf, size);
  for (uint8_t i = 0; i < len; ++i) {
    size = snprintf(sbuf, 32, "%02x", data[i]);
    usart1_tx_write(sbuf, size);
  }
  size = snprintf(sbuf, 32, "\r\n");
  usart1_tx_write(sbuf, size);
}

static void sense_begin(struct JVSIO_SenseClient* client) {
  (void)client;
  uint8_t channels[] = {ADC_CHANNEL5};
  adc_set_regular_sequence(ADC1, 1, channels);
  adc_start_conversion_direct(ADC1);
}

static void sense_set(struct JVSIO_SenseClient* client, bool ready) {
  (void)client;
  (void)ready;
  // Not used in the host mode.
}

static bool sense_isReady(struct JVSIO_SenseClient* client) {
  (void)client;
  // See sense_isConnected() below.
  // When all devices are ready, the sense is pulled to GND by the device.
  // So, assumes less than 20000 (~1.0[v]) meant they are ready.
  return sense_data < 20000;
}

static bool sense_isConnected(struct JVSIO_SenseClient* client) {
  (void)client;
  // Assumes running at 3.3[V] and ADC pin is 5[V] tolerant.
  // The sense pin should be pulled-up to 3.3+[V].
  // 65535 => 3.3(VDD) - 5.0V : Disconnected
  // The I/O drives the sense signal to 2.5V on their connections.
  // So, assumes less than 60000 meant there is a device on the bus.
  bool connected = sense_data < 60000;

  return connected;
}

static void led_begin(struct JVSIO_LedClient* client) {
  // Not used for host.
  (void)client;
}

static void led_set(struct JVSIO_LedClient* client, bool ready) {
  // Not used for host.
  (void)client;
  (void)ready;
}

static void time_delayMicroseconds(struct JVSIO_TimeClient* client,
                                   unsigned int usec) {
  (void)client;
  uint32_t t = tick + 1 + (200 + (usec << 3)) / 1000;
  while (tick < t)
    ;
}

static void time_delay(struct JVSIO_TimeClient* client, unsigned int msec) {
  (void)client;
  (void)msec;
  uint32_t t = tick + (msec << 3);
  while (tick < t)
    ;
}

static uint32_t time_getTick(struct JVSIO_TimeClient* client) {
  (void)client;
  // Returns msec from the boot.
  return tick >> 3;
}

// optional.
static void host_receiveIoId(struct JVSIO_HostClient* client,
                             uint8_t address,
                             uint8_t* data,
                             uint8_t len) {
  (void)client;
  (void)address;
  (void)data;
  (void)len;
  char sbuf[256];
  size_t size = snprintf(sbuf, 256, "ID: %s\r\n", data);
  usart1_tx_write(sbuf, size);
}

// optional.
static void host_receiveCommandRev(struct JVSIO_HostClient* client,
                                   uint8_t address,
                                   uint8_t rev) {
  (void)client;
  (void)address;
  data_dump(0, "Command Rev", &rev, 1);
}

// optional.
static void host_receiveJvRev(struct JVSIO_HostClient* client,
                              uint8_t address,
                              uint8_t rev) {
  (void)client;
  (void)address;
  data_dump(0, "Jv Rev", &rev, 1);
}

// optional.
static void host_receiveProtocolVer(struct JVSIO_HostClient* client,
                                    uint8_t address,
                                    uint8_t ver) {
  (void)client;
  (void)address;
  data_dump(0, "Protocol Ver", &ver, 1);
}

// optional.
static void host_receiveFunctionCheck(struct JVSIO_HostClient* client,
                                      uint8_t address,
                                      uint8_t* data,
                                      uint8_t len) {
  (void)client;
  (void)address;
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
  (void)client;
  // Example to dump all information.
  char sbuf[256];
  size_t size = snprintf(sbuf, 256, "%s\r\n", "--------------------");
  usart1_tx_write(sbuf, size);
  size = snprintf(sbuf, 256, "Players: %d\r\n", players);
  usart1_tx_write(sbuf, size);
  size =
      snprintf(sbuf, 256, "Test: %s\r\n", (coin_state & 0x80) ? "ON" : "OFF");
  usart1_tx_write(sbuf, size);
  for (uint8_t player = 0; player < players; ++player) {
    size = snprintf(sbuf, 256, "Coin %d: %s\r\n", player + 1,
                    (coin_state & (1 << player)) ? "ON" : "OFF");
    usart1_tx_write(sbuf, size);
  }
  size = snprintf(sbuf, 256, "%s\r\n", "  StSvUpDnLtRtP1P2P3P4P5P6P7P8P9Pa");
  usart1_tx_write(sbuf, size);
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
    usart1_tx_write(sbuf, size);
  };
}

void JVS_HOST_Init() {
  rcc_periph_clock_enable(RCC_GPIOA);

  usart1_tx_init();

  // Need to adjust the value so that the interrupt occurs in every 1/8 ms.
  systick_set_reload(1000);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();
  systick_interrupt_enable();

  // Setup UART2 and GPIO for JVS RS584 communication.
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
  gpio_clear(GPIOA, GPIO1);

  rcc_periph_clock_enable(RCC_USART2);

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART2_TX);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

  usart_enable(USART2);
  nvic_enable_irq(NVIC_USART2_IRQ);

  // Setup ADC for JVS sense.
  rcc_periph_clock_enable(RCC_ADC1);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO5);
  adc_power_off(ADC1);

  adc_disable_scan_mode(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_left_aligned(ADC1);
  adc_set_sample_time(ADC1, ADC_CHANNEL5, ADC_SMPR_SMP_239DOT5CYC);

  adc_power_on(ADC1);

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
  if (adc_eoc(ADC1)) {
    sense_data = adc_read_regular(ADC1);
    adc_start_conversion_direct(ADC1);
  }
  if (io->host(io, &host_client)) {
    // Call `sync` while the bus is ready, or once per frame.
    // `synced` will be called when the response is parsed.
    io->sync(io);
  }
}