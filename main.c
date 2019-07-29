/*****************************************************************************
 *   PPM2DSM - Convert PPM signal to serial signal compatible with DSM2 / DSMx
 *             modules
 *
 */

/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*  This version has been tested with STM32F030 F4P6
 *
 *  HW connections to STM32F030 are as this:
 *    PA0   LED output (active low, connect RED LED in series ~1kOhm to Vdd)
 *    PA1   Bind button input, active low, connect to pushbutton to ground
 *    PA2   Serial output
 *    PA9   PPM input
 *
 *  DSM2 serial frame format is this:
 *    header:
 *      byte 1: 0x80 == BIND mode, 0x00 = NORMAL mode
 *      byte 2: 0x00
 *
 *    channel data:
 *      byte 1:
 *        bit 0-1:  Bits 8-9 of channel data
 *        bit 2-4:  Channel number
 *      byte 2:
 *        bit 0-7:  Bits 0-7 of channel data
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f0xx.h"
#include "usart_stm32f0xx.h"

//--- Defines

// maximum number of channels we can decode, don't increase this above 8

#define MAX_CHANNELS      6

// max number of DSM2 Channels transmitted. Some tranceiver  modules 
// can't handle mor than six channels. Don't increase this above 8

#define DSM2_CHANNELS     6

// DSM2 serial output baud rate, has to be changed for other modules

#define DSM2_BAUD         125000                // Output baud rate

// Uncomment next line if your sender is using an inverted PPM signal

//#define PPM_INV           1

// Valid frames before output is not failsafe

#define VALID_FRAMES      2

// CPU clock - change if you changed the clock config

#define F_CPU             32000000

// Internal - modify at your own risk

#define TIMER_PRESCALER   (F_CPU / 1000000 - 1) // Timer prescaler, tick = 1us
#define PULSE_LENGTH_MIN  750                   // Min ticks of valid pulse
#define PULSE_LENGTH_MAX  2250                  // Max ticks of valid pulse
#define SYNC_LENGTH_MIN   5000                  // Min ticks of sync gap
#define PULSE_TIMEOUT     50000                 // Pulse timeout

#define LED_OFF         { led_blink =  0; GPIOA->ODR |= GPIO_ODR_0; }
#define LED_ON          { led_blink = 0; GPIOA->ODR &= ~GPIO_ODR_0; }
#define LED_BLINK_SLOW  { led_blink = 10; }
#define LED_BLINK_FAST  { led_blink = 3; }


// Internal states of the frame acquisition

typedef enum {
  S_WAIT, S_OK, S_FAILSAFE
} state_t;

// Functions

void ppm2dsm(uint16_t ppm[], uint8_t *dsm, int num_chan, uint8_t bindmode, const int chan_map[]);
void send_dsm(uint8_t *dsm, int num_chan);

void printf_hex(uint8_t b);
void init_hw(void);

//--- Data

// Failsafe data for the channels, set to minimum for ch#1 (throttle),
// all other to neutral

static uint16_t frame_data_failsafe[MAX_CHANNELS] = {
  1000, 1500, 1500, 1500, 1500, 1500
};

// Channel mapping input -> output

static const int chan_map[DSM2_CHANNELS] = {0, 1, 2, 3, 4, 5};

// Misc...

volatile static state_t ppm_state = S_WAIT; // Internal state
volatile static int frames_read = 0;        // Num of consecutive valid frames read
volatile static int chan_detected = 0;      // Number of channels detected
volatile static uint16_t frame_data[MAX_CHANNELS]; // Frame data (pulse length)
volatile static uint8_t frame_data_lock = 0;    // Lock flag
volatile static uint8_t frame_data_new = 0;     // New data flag
volatile static uint16_t led_blink = 0;     // LED blink frequency (1=highest, 0=no)
volatile static uint8_t bind_mode = 0;      // Flag for BIND mode
volatile static uint16_t timer_counter = 0; // 0.1s counter for timing purposes

// Here we go

void main(void) {
  static uint8_t dsm2_data[2+DSM2_CHANNELS*2];
  uint16_t timer_last = 0;

  // Some initialization

  init_hw();
  usart_init(USART1, DSM2_BAUD);

  // Turn LED OFF

  LED_OFF;

  // Main loop

  while(1) {
    if (chan_detected != 0) {

      // create dsm frame for either failsafe data each 0.1s or
      // the real ppm data if new frame available

      if (ppm_state == S_FAILSAFE && timer_counter != timer_last) {
        ppm2dsm(frame_data_failsafe, dsm2_data, chan_detected, bind_mode, chan_map);
        timer_last = timer_counter;
        send_dsm(dsm2_data, chan_detected);
      }
      else if (ppm_state == S_OK && frame_data_new) {
        frame_data_lock = 1;
        ppm2dsm((uint16_t *)frame_data, dsm2_data, chan_detected, bind_mode, chan_map);
        frame_data_new = 0;
        frame_data_lock = 0;
        send_dsm(dsm2_data, chan_detected);
      }
    }

    // set LED states

    if (bind_mode) {
      LED_BLINK_FAST;
    }
    else if (ppm_state == S_OK) {
      LED_ON;
    }
    else if (ppm_state == S_FAILSAFE) {
      LED_BLINK_SLOW;
    }
    else {
      LED_OFF;
    }
  }
}


/** \brief Compute DSM2 data from ppm
 */
void ppm2dsm(uint16_t ppm[], uint8_t *dsm, int num_chan, uint8_t bindmode, const int chan_map[]) {
  uint16_t ppm_chan;

  // Write header

  if (bindmode) {
    *dsm++ = 0x80;
  }
  else {
    *dsm++ = 0x00;
  }

  *dsm++ = 0x00;

  // Write channel data

  for (int i = 0; i < num_chan; i++) {
    ppm_chan = ppm[chan_map[i]]-1000;
    ppm_chan &= 0x3ff;
    *dsm++ = (i << 2) | (ppm_chan >> 8);
    *dsm++ = ppm_chan & 0xff;
  }
}

/** \brief Send DSM data
 */
void send_dsm(uint8_t *dsm, int num_chan) {
  #ifdef DEBUG
  printf("Channels: %d # State: %d # Frames: %d # DSM2 data:",
  chan_detected, ppm_state, frames_read);
  for (int i = 0; i < 2+2*num_chan; i++) {
    putchar(' ');
    printf_hex(dsm[i]);
  }
  printf("\n");
  #endif

  usart_write(USART1, dsm, 2+2*num_chan);
}

void printf_hex(uint8_t b) {
    uint8_t b1 = (b >> 4) & 0x0F;
    uint8_t b2 = (b & 0x0F);
    putchar((b1 < 10) ? ('0' + b1) : 'a' + b1 - 10);
    putchar((b2 < 10) ? ('0' + b2) : 'a' + b2 - 10);
}


/** \brief Initialize hardware
 */
void init_hw(void) {

  // RCC

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;    // Enable GPIO A
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   // Enable TIM1
  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;  // Enable TIM14

  // Debug MCU

  #ifdef DEBUG
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
  DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP; // Stop TIM1 in DEBUG
  #endif

  // GPIO

  GPIOA->MODER |= GPIO_MODER_MODER0_0;  // Set PA0 to output for LED
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR9_0;  // Enable pullups

  GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER9_1; // Enable alternate functions
  GPIOA->AFR[0] |= (1<<8);              // PA2= AF1 / USART1 TX
  GPIOA->AFR[1] |= (2<<4);              // PA9= AF2 / TIM1_CH2

  // Set unused pins to PULLUP

  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR3_0|GPIO_PUPDR_PUPDR4_0|GPIO_PUPDR_PUPDR5_0|
    GPIO_PUPDR_PUPDR6_0|GPIO_PUPDR_PUPDR7_0|GPIO_PUPDR_PUPDR8_0|GPIO_PUPDR_PUPDR10_0;

  // TIM1
  // Timer is configured so that we capture rising (falling) edge on input
  // At the same time the timer is reset to 0, so we get exact pulse time.

  TIM1->CCMR1 |= TIM_CCMR1_CC2S_0;      // InputCapture2 from TI2 channel
  #ifdef PPM_INV
  TIM1->CCER |= TIM_CCER_CC2P;          // inverted PPM signal
  #endif
  TIM1->CCER |= TIM_CCER_CC2E;          // Enable capture
  TIM1->DIER |= TIM_DIER_CC2IE | TIM_DIER_UIE; // Enable intrerrupts
  TIM1->PSC = TIMER_PRESCALER;          // Set prescaler
  TIM1->SMCR |= TIM_SMCR_SMS_2 |
    TIM_SMCR_TS_2 | TIM_SMCR_TS_1;      // Reset timer on capture 
  TIM1->ARR = PULSE_TIMEOUT - 1;          // Auto Reload after this time
  TIM1->CR1 |= TIM_CR1_URS | TIM_CR1_CEN; // Enable timer

  NVIC_SetPriority(TIM1_CC_IRQn, 0);

  // TIM14
  // Timer is configured to do overrun each 1/10s for blinking purposes and failsafe sending

  TIM14->PSC = F_CPU / 1000 - 1;        // set prescaler to 1ms
  TIM14->ARR = 99;
  TIM14->DIER |= TIM_DIER_UIE;
  TIM14->CR1 |= TIM_CR1_CEN;            // Enable timer

  NVIC_SetPriority(TIM14_IRQn, 1);

  // Enable interrupts

  NVIC_EnableIRQ(TIM1_CC_IRQn);         // Enable
  NVIC_EnableIRQ(TIM14_IRQn);
}

// TIM14 interrupt handles LED blinking and BIND switch

void TIM14_IRQHandler(void) {
  if (TIM14->SR & TIM_SR_UIF) {
    TIM14->SR &= ~TIM_SR_UIF;

    timer_counter++;
    if (led_blink != 0) {
      if ((timer_counter % led_blink) == 0) {
        GPIOA->ODR ^= GPIO_ODR_0;
      }
    }

    if (GPIOA->IDR & GPIO_IDR_1) {
      bind_mode = 0;
    }
    else {
      bind_mode = 1;
    }
  }
}


// TIM1 interrupt handles the pulses

void TIM1_CC_IRQHandler(void) {
  static int pulses_read = 0;             // Number of pulses read
  static uint16_t pulses[MAX_CHANNELS];   // Actual pulses
  static int chan_last = 0;               // Last number of channels detected
  uint8_t frame_ok = 0;

  int pulse_length;

  // Check if we have an overrun (means timeout)

  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR &= ~TIM_SR_UIF;

    if (ppm_state != S_WAIT) {

      // We have failsafe situation

      ppm_state = S_FAILSAFE;
      frames_read = 0;
      pulses_read = 0;
    }
  }

  // Check if a pulse was captured

  if (TIM1->SR & TIM_SR_CC2IF) {

    pulse_length = TIM1->CCR2;

    if (pulse_length >= PULSE_LENGTH_MIN &&   // Check if valid pulse
        pulse_length <= PULSE_LENGTH_MAX) {

      if (pulses_read < MAX_CHANNELS) {       // Store pulse length
        pulses[pulses_read] = pulse_length;
      }
      pulses_read++;
    }
    else if (pulse_length >= SYNC_LENGTH_MIN) { // Check if SYNC
      if (pulses_read == chan_last) {         // Num. of channels constant?
        frames_read++;
        frame_ok = 1;
      }
      else {
        frames_read = 0;
        if(pulses_read <= MAX_CHANNELS) {
          chan_last = pulses_read;
        }
        else {
          chan_last = 0;
        }
      }
      pulses_read = 0;
    }
    else {                                    // Invalid pulse
      frames_read = 0;
      chan_last = 0;
      pulses_read = 0;
    }

    // Check if we have num of channels detected

    if (chan_detected == 0) {
      if (frames_read >= VALID_FRAMES) {
        chan_detected = chan_last;
        ppm_state = S_FAILSAFE;
      }
    }
    else {
      if (chan_last != chan_detected) {
        ppm_state = S_FAILSAFE;
      }
      else if (frame_ok) {
        ppm_state = S_OK;
        if (frame_data_lock == 0) {
          memcpy((void *)frame_data, pulses, sizeof(pulses));
          frame_data_new = 1;
        }
      }
    }
  }
}
