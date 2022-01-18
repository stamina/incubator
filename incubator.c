/*

MIT License

Copyright (c) 2018 Bas Brugman
www.visionnaire.nl
https://github.com/stamina

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "i2cmaster.h"
#include "incubator.h"
#include "spi.h"
#include "ssd1306.h"
#include "usart.h"

void init_mcu(void);
void sensor_init(uint8_t addr);
void sensor_read(tSensortempr *sensor);
void save_tempr(void);
void process_tempr(void);
float avg_sensors(void);
float avg_readings(float *readings, uint8_t idx);
void graph_readings(float *readings, uint8_t idx);
float avg_cumulative(void);
void display_oled(void);
#ifdef DEBUG_TRACE
void trace_tempr(void);
#endif
void init_states(void);
void set_state(uint8_t state);
void reset_minmax(void);
void check_encoder_btn(void);
void btn_pressed(void);
void btn_rotate(uint8_t dir);
void setup_interrupts(void);
void start_sec_timer(void);
void start_ms_timer(void);

void start_sec_timer() {
  // Configure timer 1 (16-bit) for CTC mode (Clear on Timer Compare)
  TCCR1A &= ~((1 << WGM11) | (1 << WGM10));  // CTC Mode
  TCCR1B |= (1 << WGM12);                    // CTC Mode
  // prescale 1024
  TCCR1B |= ((1 << CS10) | (1 << CS12));
  TCCR1B &= ~(1 << CS11);
  TIMSK1 |= (1 << OCIE1A);  // Enable Compare A interrupt
  OCR1A = 19531;  // Set CTC compare value, default approx. every second
}

void start_ms_timer() {
  // Configure timer 3 (16-bit) for CTC mode (Clear on Timer Compare)
  TCCR3A &= ~((1 << WGM31) | (1 << WGM30));  // CTC Mode
  TCCR3B |= (1 << WGM32);                    // CTC Mode
  // prescale with 8 to get 2.5million hz with a 20mhz crystal
  TCCR3B &= ~((1 << CS30) | (1 << CS32));
  TCCR3B |= (1 << CS31);
  TIMSK3 |= (1 << OCIE3A);  // Enable Compare A interrupt
  OCR3A = 2499;  // Set CTC compare value, every 1ms (counts from 0 to 2499
                 // -> 2.5million hz / 2500 = 1000)
}

void setup_interrupts() {
  EIMSK = 1 << INT0;                     // enable external INT0
  EICRA |= (1 << ISC01) | (1 << ISC00);  // INT0 sample rising edge
}

// 1 sec timer
ISR(TIMER1_COMPA_vect) {
  FLAG_SECOND_PASSED = 1;
  SCREEN0_TIMER++;
  SCREEN_SAVER_TIMER++;  // go to screen0 splash animation after 10mins
  if (BUSY_WITH_MSG > 0) {
    BUSY_WITH_MSG--;
  }
}

// 1 ms timer
ISR(TIMER3_COMPA_vect) {
  if (BUSY_WITH_ROTATE > 0) {
    BUSY_WITH_ROTATE--;
  }
}

// PD2 pin external trigger rising edge
ISR(INT0_vect) {
  if (STATUS_LEFT) {
    btn_rotate(CYCLE_LEFT);
  } else {
    btn_rotate(CYCLE_RIGHT);
  }
}

void sensor_init(uint8_t addr) {
  // set to 12bits accuracy for sensor
  i2c_start_wait(addr + I2C_WRITE);  // set device address and write mode
  i2c_write(
      CONF_REG);  // write pointer register 00000001 to select config register
  i2c_rep_start(addr + I2C_READ);  // set device address and read mode
  g_cur_conf_reg = i2c_readNak();  // read config register
  i2c_stop();
  g_cur_conf_reg |= g_my_conf_reg;  // set config register for 12 bit resolution
  i2c_start_wait(addr + I2C_WRITE);  // set device address and write mode
  i2c_write(
      CONF_REG);  // write pointer register 00000001 to select config register
  i2c_write(g_cur_conf_reg);  // write config back to config register
  i2c_stop();
}

// read temperature of sensor
void sensor_read(tSensortempr *sensor) {
  i2c_start_wait(sensor->addr +
                 I2C_WRITE);  // set device address and write mode
  i2c_write(
      TEMPR_REG);  // write pointer register 00000000 to select temp register
  i2c_rep_start(sensor->addr + I2C_READ);  // set device address and read mode
  g_high_byte = i2c_readAck();  // read high byte of temperature -> counts
                                // before decimal
  g_low_byte = i2c_readNak();   // read low byte of temperature
  i2c_stop();
  g_tempr = (((g_high_byte << 8) | g_low_byte) >> 4);
  sensor->cur = (float)g_tempr / 16;
}

void init_mcu() {
  MCUCR = 0x80;  // disable JTAG at runtime (2 calls in a row needed)
  MCUCR = 0x80;
  wdt_enable(WDTO_2S);  // enable 2 sec watchdog
  sei();                // enable global interrupts
  // rotary encoder sense pins DDR/PORT: as input(0)/high(1)
  DDRD &= ~(1 << PIND4);  // btn press
  PORTD |= (1 << PIND4);
  DDRD &= ~(1 << PIND2);  // right turn
  PORTD |= (1 << PIND2);
  DDRD &= ~(1 << PIND3);  // left turn
  PORTD |= (1 << PIND3);
  setup_interrupts();
  uart_init();
  i2c_init();
  start_ms_timer();
  start_sec_timer();
  // 10 Mhz SPI, as fast as possible with a 20mhz external crystal
  spi_init(SPI_CLOCK_DIV2_2X, SPI_MODE0, SPI_MSBFIRST, SPI_NO_INTERRUPT);
  oled_init();
  for (uint8_t i = 0; i < SENSOR_NR; i++) {
    sensor_init(g_sensors[i].addr);
  }
  g_state = TEMPR_SCREEN0;
  g_graph_pollrate = POLLRATE_SEC;
  g_screen1_sample = SCREEN1_SAMPLE_CUR;
}

// store the sensor data every second
void save_tempr() {
  for (uint8_t i = 0; i < SENSOR_NR; i++) {
    sensor_read(&g_sensors[i]);
    // check min/max
    if (g_sensors[i].cur > g_sensors[i].max) {
      g_sensors[i].max = g_sensors[i].cur;
    }
    if ((g_sensors[i].cur < g_sensors[i].min) || (g_sensors[i].min == 0)) {
      g_sensors[i].min = g_sensors[i].cur;
    }
  }
  process_tempr();
}

void reset_minmax() {
  for (uint8_t i = 0; i < SENSOR_NR; i++) {
    g_sensors[i].max = 0;
    g_sensors[i].min = 0;
  }
}

void process_tempr() {
  g_hatchday = (g_hour_cnt / 24) + 1;
  if (g_history_tempr.idx_sec > AVG_TEMPR_READINGS - 1) {  // rollover
    g_history_tempr.idx_sec = 0;
  }
  // put a new second in
  g_history_tempr.cur = g_history_tempr.readings_sec[g_history_tempr.idx_sec] =
      avg_sensors();
  if ((g_history_tempr.cur < g_history_tempr.min) ||
      (g_history_tempr.min == 0)) {
    g_history_tempr.min = g_history_tempr.cur;
  }
  if (g_history_tempr.cur > g_history_tempr.max) {
    g_history_tempr.max = g_history_tempr.cur;
  }
  g_history_tempr.avg = avg_cumulative();
  g_history_tempr.idx_sec++;
  g_sec_cnt++;
  if (g_sec_cnt >= 60) {
    g_sec_cnt = 0;
    // put a new minute in
    if (g_history_tempr.idx_min > AVG_TEMPR_READINGS - 1) {  // rollover
      g_history_tempr.idx_min = 0;
    }
    g_history_tempr.readings_min[g_history_tempr.idx_min] =
        avg_readings(g_history_tempr.readings_sec, g_history_tempr.idx_sec);
    g_history_tempr.idx_min++;
    g_min_cnt++;
    if (g_min_cnt >= 60) {
      g_min_cnt = 0;
      // put a new hour in
      if (g_history_tempr.idx_hour > AVG_TEMPR_READINGS - 1) {  // rollover
        g_history_tempr.idx_hour = 0;
      }
      g_history_tempr.readings_hour[g_history_tempr.idx_hour] =
          avg_readings(g_history_tempr.readings_min, g_history_tempr.idx_min);
      g_history_tempr.idx_hour++;
      g_hour_cnt++;
    }
  }
}

// average of the 4 sensors
float avg_sensors() {
  float total = 0;
  for (uint8_t i = 0; i < SENSOR_NR; i++) {
    total += g_sensors[i].cur;
  }
  return (total / SENSOR_NR);
}

// average of 60 samples, reading backwards
float avg_readings(float *readings, uint8_t idx) {
  if (!idx) {
    return 0;  // this should never happen
  }
  float total = 0;
  uint8_t cur_idx = idx - 1;
  for (uint8_t i = 0; i < 60; i++, cur_idx--) {
    total += readings[cur_idx];
    // flip to end of array when index 0 reached
    if (!cur_idx) {
      cur_idx = AVG_TEMPR_READINGS - 1;
    }
  }
  return (total / 60);
}

// display all 80 graph readings, reading ring buffer backwards
// starting index is always current idx-1
// loop ends when idx is read
void graph_readings(float *readings, uint8_t idx) {
  uint8_t cnt = 0;
  if (!idx) {  // rollover
    idx = AVG_TEMPR_READINGS - 1;
  } else {
    idx = idx - 1;
  }
  do {
    oled_fill_graph(cnt, readings[idx]);
    if (!idx) {
      idx = AVG_TEMPR_READINGS - 1;
    } else {
      idx--;
    }
    cnt++;
  } while (cnt < AVG_TEMPR_READINGS - 1);
}

// returns the biggest cumulative/continuous avg
// first check the possible 80 hour samples
// then check the possible 80 minute samples
// finally check the possible 80 second samples
float avg_cumulative() {
  float total = 0;
  uint8_t num = 0;
  for (uint8_t i = 0; i < AVG_TEMPR_READINGS; i++) {
    if (g_history_tempr.readings_hour[i] > 0) {
      num++;
      total += g_history_tempr.readings_hour[i];
    }
  }
  if (num) {
    return total / num;
  } else {
    for (uint8_t i = 0; i < AVG_TEMPR_READINGS; i++) {
      if (g_history_tempr.readings_min[i] > 0) {
        num++;
        total += g_history_tempr.readings_min[i];
      }
    }
  }
  if (num) {
    return total / num;
  } else {
    for (uint8_t i = 0; i < AVG_TEMPR_READINGS; i++) {
      if (g_history_tempr.readings_sec[i] > 0) {
        num++;
        total += g_history_tempr.readings_sec[i];
      }
    }
  }
  if (num) {
    return total / num;
  }
  return 0;
}

#ifdef DEBUG_TRACE
void trace_tempr() {
  uart_put_str("S1 cur: ");
  sprintf(g_tempr_display, "%f", g_sensors[0].cur);
  uart_put_str(g_tempr_display);
  uart_put_str(" S1 min: ");
  sprintf(g_tempr_display, "%f", g_sensors[0].min);
  uart_put_str(g_tempr_display);
  uart_put_str(" S1 max: ");
  sprintf(g_tempr_display, "%f", g_sensors[0].max);
  uart_put_str(g_tempr_display);
  uart_put('\n');
  uart_put_str("S2 cur: ");
  sprintf(g_tempr_display, "%f", g_sensors[1].cur);
  uart_put_str(g_tempr_display);
  uart_put_str(" S2 min: ");
  sprintf(g_tempr_display, "%f", g_sensors[1].min);
  uart_put_str(g_tempr_display);
  uart_put_str(" S2 max: ");
  sprintf(g_tempr_display, "%f", g_sensors[1].max);
  uart_put_str(g_tempr_display);
  uart_put('\n');
  uart_put_str("S3 cur: ");
  sprintf(g_tempr_display, "%f", g_sensors[2].cur);
  uart_put_str(g_tempr_display);
  uart_put_str(" S3 min: ");
  sprintf(g_tempr_display, "%f", g_sensors[2].min);
  uart_put_str(g_tempr_display);
  uart_put_str(" S3 max: ");
  sprintf(g_tempr_display, "%f", g_sensors[2].max);
  uart_put_str(g_tempr_display);
  uart_put('\n');
  uart_put_str("S4 cur: ");
  sprintf(g_tempr_display, "%f", g_sensors[3].cur);
  uart_put_str(g_tempr_display);
  uart_put_str(" S4 min: ");
  sprintf(g_tempr_display, "%f", g_sensors[3].min);
  uart_put_str(g_tempr_display);
  uart_put_str(" S4 max: ");
  sprintf(g_tempr_display, "%f", g_sensors[3].max);
  uart_put_str(g_tempr_display);
  uart_put('\n');
  uart_put_str("g_history_tempr cur: ");
  sprintf(g_tempr_display, "%f", g_history_tempr.cur);
  uart_put_str(g_tempr_display);
  uart_put('\n');
  uart_put_str("g_history_tempr avg: ");
  sprintf(g_tempr_display, "%f", g_history_tempr.avg);
  uart_put_str(g_tempr_display);
  uart_put('\n');
  uart_put_str("g_history_tempr min: ");
  sprintf(g_tempr_display, "%f", g_history_tempr.min);
  uart_put_str(g_tempr_display);
  uart_put('\n');
  uart_put_str("g_history_tempr max: ");
  sprintf(g_tempr_display, "%f", g_history_tempr.max);
  uart_put_str(g_tempr_display);
  uart_put('\n');
  uart_put_str("g_history_tempr readings_sec: ");
  for (uint8_t i = 0; i < AVG_TEMPR_READINGS; i++) {
    sprintf(g_tempr_display, "%f", g_history_tempr.readings_sec[i]);
    uart_put_str(g_tempr_display);
    if (i < AVG_TEMPR_READINGS - 1) {
      uart_put_str(", ");
    }
  }
  uart_put('\n');
  uart_put_str("g_history_tempr readings_min: ");
  for (uint8_t i = 0; i < AVG_TEMPR_READINGS; i++) {
    sprintf(g_tempr_display, "%f", g_history_tempr.readings_min[i]);
    uart_put_str(g_tempr_display);
    if (i < AVG_TEMPR_READINGS - 1) {
      uart_put_str(", ");
    }
  }
  uart_put('\n');
  uart_put_str("g_history_tempr readings_hour: ");
  for (uint8_t i = 0; i < AVG_TEMPR_READINGS; i++) {
    sprintf(g_tempr_display, "%f", g_history_tempr.readings_hour[i]);
    uart_put_str(g_tempr_display);
    if (i < AVG_TEMPR_READINGS - 1) {
      uart_put_str(", ");
    }
  }
  uart_put('\n');
}
#endif

void init_states() {
  g_screen0_idx = 0;
  g_screen1_static = g_screen2_static = 0;
  SCREEN0_TIMER = 0;
  SCREEN_SAVER_TIMER = 0;
}

void set_state(uint8_t state) {
  init_states();
  g_state = state;
}

void check_encoder_btn() {
  if (!(STATUS_BTN)) {
    btn_pressed();
#ifdef DEBUG_TRACE
    uart_put_str("BTN PRESS");
    uart_put('\n');
#endif
  }
}

void btn_pressed() {
  BUSY_WITH_MSG = 2;  // 2 seconds for message displays
#ifdef DEBUG_TRACE
  uart_put_str("state in btn_pressed: ");
  uart_put_int(g_state);
  uart_put('\n');
#endif
  switch (g_state) {
    case TEMPR_SCREEN0:
      oled_send_msg("    Resetting device");
      // reboot mcu, let watchdog run out
      _delay_ms(3000);
      break;
    case TEMPR_SCREEN1:
      switch (g_screen1_sample) {
        case SCREEN1_SAMPLE_CUR:
          g_screen1_sample = SCREEN1_SAMPLE_AVG;
          oled_send_msg(" Average temperature");
          break;
        case SCREEN1_SAMPLE_AVG:
          g_screen1_sample = SCREEN1_SAMPLE_MIN;
          oled_send_msg(" Minimum temperature");
          break;
        case SCREEN1_SAMPLE_MIN:
          g_screen1_sample = SCREEN1_SAMPLE_MAX;
          oled_send_msg(" Maximum temperature");
          break;
        case SCREEN1_SAMPLE_MAX:
          g_screen1_sample = SCREEN1_SAMPLE_CUR;
          oled_send_msg(" Current temperature");
          break;
      }
      break;
    case TEMPR_SCREEN2:
      switch (g_graph_pollrate) {
        case POLLRATE_SEC:
          g_graph_pollrate = POLLRATE_MIN;
          oled_send_msg(" Sample every minute");
          break;
        case POLLRATE_MIN:
          g_graph_pollrate = POLLRATE_HOUR;
          oled_send_msg("   Sample every hour");
          break;
        case POLLRATE_HOUR:
          g_graph_pollrate = POLLRATE_SEC;
          oled_send_msg(" Sample every second");
          break;
      }
      break;
    case TEMPR_SCREEN3:
      reset_minmax();
      oled_send_msg("   Resetting min/max");
      break;
  }
  init_states();  // reinit states for proper screen output
}

void btn_rotate(uint8_t dir) {
  if (BUSY_WITH_ROTATE) return;
#ifdef DEBUG_TRACE
  if (dir == CYCLE_RIGHT) {
    uart_put_str("CYCLE RIGHT\n");
  } else {
    uart_put_str("CYCLE LEFT\n");
  }
#endif
  BUSY_WITH_ROTATE = 50;
  switch (g_state) {
    case TEMPR_SCREEN0:
      if (dir == CYCLE_RIGHT) {
        set_state(TEMPR_SCREEN1);
      } else {
        set_state(TEMPR_SCREEN3);
      }
      break;
    case TEMPR_SCREEN1:
      if (dir == CYCLE_RIGHT) {
        set_state(TEMPR_SCREEN2);
      } else {
        set_state(TEMPR_SCREEN0);
      }
      break;
    case TEMPR_SCREEN2:
      if (dir == CYCLE_RIGHT) {
        set_state(TEMPR_SCREEN3);
      } else {
        set_state(TEMPR_SCREEN1);
      }
      break;
    case TEMPR_SCREEN3:
      if (dir == CYCLE_RIGHT) {
        set_state(TEMPR_SCREEN0);
      } else {
        set_state(TEMPR_SCREEN2);
      }
      break;
  }
  // immediately show screen
  if (!BUSY_WITH_MSG) {
    display_oled();
  }
}

void display_oled() {
  if (SCREEN_SAVER_TIMER > 600) {
    set_state(TEMPR_SCREEN0);
  }
  switch (g_state) {
    case TEMPR_SCREEN0:
      if (g_screen0_idx >= 8) {
        if (SCREEN0_TIMER > 16) {  // display idx 7 (scrolling splash) longer
          SCREEN0_TIMER = 0;
          g_screen0_idx = 0;
        }
      } else {
        oled_s0_screen(g_screen0_sequence[g_screen0_idx]);
        g_screen0_idx++;
      }
      break;
    case TEMPR_SCREEN1:
      if (!g_screen1_static) {
        oled_s1_screen();
        g_screen1_static = 1;
      }
      switch (g_screen1_sample) {
        case SCREEN1_SAMPLE_CUR:
          oled_s1_tempr(g_history_tempr.cur, g_hatchday, SCREEN1_SAMPLE_CUR);
          break;
        case SCREEN1_SAMPLE_AVG:
          oled_s1_tempr(g_history_tempr.avg, g_hatchday, SCREEN1_SAMPLE_AVG);
          break;
        case SCREEN1_SAMPLE_MIN:
          oled_s1_tempr(g_history_tempr.min, g_hatchday, SCREEN1_SAMPLE_MIN);
          break;
        case SCREEN1_SAMPLE_MAX:
          oled_s1_tempr(g_history_tempr.max, g_hatchday, SCREEN1_SAMPLE_MAX);
          break;
      }
      break;
    case TEMPR_SCREEN2:
      if (!g_screen2_static) {
        oled_s2_screen();
        g_screen2_static = 1;
      }
      oled_clear_graph();
      switch (g_graph_pollrate) {
        case POLLRATE_SEC:
          graph_readings(g_history_tempr.readings_sec, g_history_tempr.idx_sec);
          break;
        case POLLRATE_MIN:
          graph_readings(g_history_tempr.readings_min, g_history_tempr.idx_min);
          break;
        case POLLRATE_HOUR:
          graph_readings(g_history_tempr.readings_hour,
                         g_history_tempr.idx_hour);
      }
      oled_draw_graph(g_history_tempr.cur);
      break;
    case TEMPR_SCREEN3:
      oled_s3_screen(g_sensors, g_history_tempr.cur);
      break;
  }
}

int main(void) {
  init_mcu();
  while (1) {
    if (!BUSY_WITH_MSG) {
      check_encoder_btn();
    }
    if (FLAG_SECOND_PASSED) {
      save_tempr();
      if (!BUSY_WITH_MSG) {
        display_oled();
      }
      FLAG_SECOND_PASSED = 0;
#ifdef DEBUG_TRACE
      trace_tempr();
#endif
    }
    if (!g_mcu_reset) {
      wdt_reset();  // reset the watchdog, i.e. don't reset the
                    // mcu
    }
  }
  return 0;
}

// EOF
