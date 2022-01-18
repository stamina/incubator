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
#ifndef _SENSORS_H_INCLUDED
#define _SENSORS_H_INCLUDED

#define AVG_TEMPR_READINGS 80

#define SCREEN1_SAMPLE_CUR 1
#define SCREEN1_SAMPLE_AVG 2
#define SCREEN1_SAMPLE_MIN 3
#define SCREEN1_SAMPLE_MAX 4

// storing average temperatures of all sensors
// last 80 readings are saved in a ring buffers
struct Historytempr {
  float readings_sec[AVG_TEMPR_READINGS];
  float readings_min[AVG_TEMPR_READINGS];
  float readings_hour[AVG_TEMPR_READINGS];
  uint8_t idx_sec;
  uint8_t idx_min;
  uint8_t idx_hour;
  float cur;
  float avg;
  float min;
  float max;
};

// storing separate sensor temperature readings
typedef struct Sensortempr {
  float cur;
  float min;
  float max;
  uint8_t addr;
} tSensortempr;

#endif
