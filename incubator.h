/*
 *
 *  Copyright (C) 2018 Bas Brugman
 *  http://www.visionnaire.nl
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */
#ifndef _INCUBATOR_H_INCLUDED
#define _INCUBATOR_H_INCLUDED

#include "sensors.h"

#define TEMPR_REG 0  // temperature register
#define CONF_REG 1   // configuration register
#define SENSOR_NR 4

// global states
#define TEMPR_SCREEN0 0
#define TEMPR_SCREEN1 1
#define TEMPR_SCREEN2 2
#define TEMPR_SCREEN3 3

#define POLLRATE_SEC 1
#define POLLRATE_MIN 2
#define POLLRATE_HOUR 3

#define CYCLE_LEFT 0
#define CYCLE_RIGHT 1

// I2C address of tmp275 (7 address bits and a direction bit, always starts with
// 1001....) using 3 physical pins A0, A1 and A2 (max 8 adressable sensors)
// all 3 address pins are grounded
#define SENSOR_ADDR1 0x90  // 0b1001000(0) LSB = READ/WRITE bit
// 2 address pins are grounded, A0 is at +5V
#define SENSOR_ADDR2 0x92  // 0b1001001(0)
// 2 address pins are grounded, A1 is at +5V
#define SENSOR_ADDR3 0x94  // 0b1001010(0)
// 2 address pins are grounded, A2 is at +5V
#define SENSOR_ADDR4 0x98  // 0b1001100(0)

// button from rotary encoder
#define STATUS_BTN PIND&(1 << PIND4)
// right rotation contact from rotary encoder
#define STATUS_RIGHT PIND&(1 << PIND2)
// left rotation contact from rotary encoder
#define STATUS_LEFT PIND&(1 << PIND3)

// configuration register of tmp275
// only 2 bits of interest are the 12bit resolution mode, ~220ms needed per
// temperature conversion, so polling 4 sensors every second is fine
uint8_t g_my_conf_reg = 0b01100000;
uint8_t g_cur_conf_reg;

tSensortempr g_sensors[SENSOR_NR] = {{0, 0, 0, SENSOR_ADDR1},
                                     {0, 0, 0, SENSOR_ADDR2},
                                     {0, 0, 0, SENSOR_ADDR3},
                                     {0, 0, 0, SENSOR_ADDR4}};

static struct Historytempr g_history_tempr;

// high and low bytes of temperature register, i.e. the value in Celsius
uint8_t g_high_byte;
uint8_t g_low_byte;
uint16_t g_tempr;
char g_tempr_display[48];
static uint8_t g_mcu_reset;
static uint8_t g_sec_cnt;
static uint16_t g_min_cnt;
static uint16_t g_hour_cnt;
static uint8_t g_hatchday;
static uint8_t g_state;
static uint8_t g_screen0_sequence[8] = {0, 1, 0, 2, 0, 3, 4, 5};
static uint8_t g_screen0_idx;
static uint8_t g_screen1_static;
static uint8_t g_screen2_static;
static uint8_t g_graph_pollrate;
static uint8_t g_screen1_sample;

static volatile uint8_t FLAG_SECOND_PASSED;
static volatile uint8_t SCREEN0_TIMER;
static volatile uint16_t SCREEN_SAVER_TIMER;
static volatile uint8_t BUSY_WITH_MSG;
static volatile uint8_t BUSY_WITH_ROTATE;

#endif
