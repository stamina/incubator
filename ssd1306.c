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
#include "ssd1306.h"
#include "bitmaps.h"
#include "fonts.h"
#include "spi.h"

void oled_init_commands(void);
void oled_start_command(void);
void oled_stop_command(void);
void oled_start_data(void);
void oled_stop_data(void);
void oled_clear_screen(void);
void oled_set_startline(uint8_t offset);
void oled_set_pos(uint8_t col_start, uint8_t col_end, uint8_t page_start,
                  uint8_t page_end);
void oled_start_left_scroll(uint8_t start_page, uint8_t stop_page);
void oled_stop_scroll(void);
uint8_t oled_calc_graph_ypos(float tempr);
void oled_write_graph_pixel(uint8_t x, uint8_t y);
void oled_write_graph_bar(uint8_t x, uint8_t y);
void oled_char_small(uint8_t chr);
void format_tempr_str(char *str, uint8_t with_dot);

char g_tempr_chars[7];

// draw dynamic part of screen 1
void oled_s1_tempr(float tempr, uint8_t hatchday, uint8_t sample) {
  uint16_t itempr = tempr * 10;
  format_tempr_str(itoa(itempr, g_tempr_chars, 10), TEMPR_WITHOUT_DOT);
  char *pchar = g_tempr_chars;
  for (uint8_t x = 0; x < strlen(g_tempr_chars); x++) {
    uint16_t offset = 0;
    uint16_t char_offset =
        224 *
        ((uint8_t)*pchar - 48);  // 224 bytes per char, 48 dec is '0' in ascii
    for (uint8_t p = 0; p < 7; p++) {
      oled_set_pos(32 * x, OLED_WIDTH - 1, p, 7);
      oled_start_data();
      for (uint16_t i = 0 + offset + char_offset; i < 32 + offset + char_offset;
           i++) {
        oled_send_data(pgm_read_byte(&g_font32x50[i]));
      }
      oled_stop_data();
      offset += 32;
    }
    pchar++;
  }
  // place 9 bytes of "dot" bitmap
  oled_set_pos(61, OLED_WIDTH - 1, 5, 7);
  oled_start_data();
  for (uint16_t i = 2240; i < 2245; i++) {
    oled_send_data(pgm_read_byte(&g_font32x50[i]));
  }
  oled_stop_data();
  // upper right corner sign
  oled_set_pos(100, OLED_WIDTH - 1, 0, 1);
  oled_start_data();
  switch (sample) {
    case SCREEN1_SAMPLE_CUR:
      for (uint16_t i = 0; i < 56; i++) {
        oled_send_data(pgm_read_byte(&g_cur_sign[i]));
      }
      break;
    case SCREEN1_SAMPLE_AVG:
      for (uint16_t i = 0; i < 56; i++) {
        oled_send_data(pgm_read_byte(&g_avg_sign[i]));
      }
      break;
    case SCREEN1_SAMPLE_MIN:
      for (uint16_t i = 0; i < 56; i++) {
        oled_send_data(pgm_read_byte(&g_min_sign[i]));
      }
      break;
    case SCREEN1_SAMPLE_MAX:
      for (uint16_t i = 0; i < 56; i++) {
        oled_send_data(pgm_read_byte(&g_max_sign[i]));
      }
      break;
  }
  oled_stop_data();
  // bottom sign (hatching days)
  oled_set_pos(0, OLED_WIDTH - 1, 6, 7);
  oled_start_data();
  for (uint16_t i = 0; i < 68; i++) {
    oled_send_data(pgm_read_byte(&g_hatch_day_sign[i]));
  }
  oled_stop_data();
  oled_set_pos(0, OLED_WIDTH - 1, 7, 7);
  oled_start_data();
  for (uint16_t i = 68; i < 136; i++) {
    oled_send_data(pgm_read_byte(&g_hatch_day_sign[i]));
  }
  oled_stop_data();
  // hatching days
  oled_set_pos(72, 127, 7, 7);
  itoa(hatchday, g_tempr_chars, 10);
  pchar = g_tempr_chars;
  oled_start_data();
  while (*pchar) {
    oled_char_small(*pchar);
    pchar++;
  }
  oled_stop_data();
}

// draw static part of screen 1
void oled_s1_screen() {
  oled_clear_screen();
  // draw background
  oled_start_data();
  for (uint16_t i = 0; i < sizeof(g_s1); i++) {
    oled_send_data(pgm_read_byte(&g_s1[i]));
  }
  oled_stop_data();
}

// draw static part of screen 2
void oled_s2_screen() {
  oled_clear_screen();
  // draw background
  oled_start_data();
  for (uint16_t i = 0; i < sizeof(g_s2); i++) {
    oled_send_data(pgm_read_byte(&g_s2[i]));
  }
  oled_stop_data();
}

void oled_s3_screen(tSensortempr *sensors, float cur) {
  char *pchar;
  uint16_t itempr;
  uint8_t page_nr = 1;
  oled_clear_screen();
  // draw background
  oled_start_data();
  for (uint16_t i = 0; i < sizeof(g_s3); i++) {
    oled_send_data(pgm_read_byte(&g_s3[i]));
  }
  oled_stop_data();
  // loop over the 4 fixed sensors
  for (uint8_t x = 0; x < 4; x++) {
    itempr = sensors[x].cur * 10;
    format_tempr_str(itoa(itempr, g_tempr_chars, 10), TEMPR_WITH_DOT);
    pchar = g_tempr_chars;
    oled_set_pos(6, 127, page_nr, page_nr);
    oled_start_data();
    while (*pchar) {
      oled_char_small(*pchar);
      pchar++;
    }
    oled_stop_data();
    itempr = sensors[x].min * 10;
    format_tempr_str(itoa(itempr, g_tempr_chars, 10), TEMPR_WITH_DOT);
    pchar = g_tempr_chars;
    oled_set_pos(36, 127, page_nr, page_nr);
    oled_start_data();
    while (*pchar) {
      oled_char_small(*pchar);
      pchar++;
    }
    oled_stop_data();
    itempr = sensors[x].max * 10;
    format_tempr_str(itoa(itempr, g_tempr_chars, 10), TEMPR_WITH_DOT);
    pchar = g_tempr_chars;
    oled_set_pos(66, 127, page_nr, page_nr);
    oled_start_data();
    while (*pchar) {
      oled_char_small(*pchar);
      pchar++;
    }
    oled_stop_data();
    page_nr += 2;
  }
  // upper right corner text
  itempr = cur * 10;
  format_tempr_str(itoa(itempr, g_tempr_chars, 10), TEMPR_WITH_DOT);
  pchar = g_tempr_chars;
  oled_set_pos(104, 127, 0, 2);
  oled_start_data();
  while (*pchar) {
    oled_char_small(*pchar);
    pchar++;
  }
  oled_stop_data();
}

// pos is the x-pos seen from the right, 0 is most right
void oled_fill_graph(uint8_t pos, float tempr) {
  uint8_t x = OLED_GRAPH_WIDTH - 1 - pos;
  uint8_t y = oled_calc_graph_ypos(tempr);
  oled_write_graph_bar(x, y);
}

// put pixel (only 1 bit set in a byte) at the right spot in the graph
// buffer depends on 8 PAGES and the width of the graph box
void oled_write_graph_pixel(uint8_t x, uint8_t y) {
  g_graph_buffer[(uint16_t)(x + (y / 8) * OLED_GRAPH_WIDTH)] |= (1 << (y & 7));
}

// vertical one pixel wide bar looped across pages
void oled_write_graph_bar(uint8_t x, uint8_t y) {
  uint8_t page = (y / 8);
  static uint8_t bar_tops[8] = {0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80};
  uint8_t bar_top = bar_tops[y % 8];
  g_graph_buffer[(uint16_t)(x + page * OLED_GRAPH_WIDTH)] = bar_top;
  for (uint8_t i = page + 1; i < 8; i++) {
    g_graph_buffer[(uint16_t)(x + i * OLED_GRAPH_WIDTH)] = 0xFF;
  }
}

void oled_draw_graph(float cur) {
  oled_set_pos(3, OLED_GRAPH_WIDTH + 2, 0, 7);  // proper position of graph
                                                // area, so we still see the
                                                // left y-axis from the
                                                // background buffer
  oled_start_data();
  for (uint16_t i = 0; i < sizeof(g_graph_buffer); i++) {
    oled_send_data(g_graph_buffer[i]);
  }
  oled_stop_data();
  uint16_t itempr;
  char *pchar;
  // upper right corner text
  oled_set_pos(104, 127, 0, 2);
  itempr = cur * 10;
  format_tempr_str(itoa(itempr, g_tempr_chars, 10), TEMPR_WITH_DOT);
  pchar = g_tempr_chars;
  oled_set_pos(104, 127, 0, 2);
  oled_start_data();
  while (*pchar) {
    oled_char_small(*pchar);
    pchar++;
  }
  oled_stop_data();
}

void oled_s0_screen(uint8_t part) {
  oled_clear_screen();
  switch (part) {
    case 0:
      oled_start_data();
      for (uint16_t i = 0; i < sizeof(g_s0_0); i++) {
        oled_send_data(pgm_read_byte(&g_s0_0[i]));
      }
      oled_stop_data();
      break;
    case 1:
      oled_start_data();
      for (uint16_t i = 0; i < sizeof(g_s0_1); i++) {
        oled_send_data(pgm_read_byte(&g_s0_1[i]));
      }
      oled_stop_data();
      break;
    case 2:
      oled_start_data();
      for (uint16_t i = 0; i < sizeof(g_s0_2); i++) {
        oled_send_data(pgm_read_byte(&g_s0_2[i]));
      }
      oled_stop_data();
      break;
    case 3:
      oled_start_data();
      for (uint16_t i = 0; i < sizeof(g_s0_3); i++) {
        oled_send_data(pgm_read_byte(&g_s0_3[i]));
      }
      oled_stop_data();
      break;
    case 4:
      oled_start_data();
      for (uint16_t i = 0; i < sizeof(g_s0_4); i++) {
        oled_send_data(pgm_read_byte(&g_s0_4[i]));
      }
      oled_stop_data();
      break;
    case 5:
      oled_start_data();
      for (uint16_t i = 0; i < sizeof(g_s0_5); i++) {
        oled_send_data(pgm_read_byte(&g_s0_5[i]));
      }
      oled_stop_data();
      oled_start_left_scroll(0x02, 0x05);
      break;
  }
}

void oled_send_msg(char *str) {
  oled_clear_screen();
  oled_set_pos(0, 127, 3, 7);
  oled_start_data();
  while (*str) {
    oled_char_small(*str);
    str++;
  }
  oled_stop_data();
}

static const uint8_t PROGMEM g_oled_init_commands[] = {
    SSD1306_DISPLAYOFF,  // display off

    SSD1306_SETDISPLAYCLOCKDIV,
    0x80,  // set to default ratio/osc frequency

    SSD1306_SETMULTIPLEX,
    OLED_HEIGHT - 1,  // Reset to default MUX

    SSD1306_SETDISPLAYOFFSET,
    0x00,  // no offset

    SSD1306_SETSTARTLINE | 0x00,  // first line to start scanning from

    SSD1306_CHARGEPUMP,
    0x14,  // enable charge pump

    SSD1306_MEMORYMODE,
    HORIZONTAL_ADDRESSING_MODE,  // horizontal addressing

    SSD1306_SEGREMAP | 0x01,  // reverse mapping, 0x00 is normal mapping

    SSD1306_COMSCANDEC,  // scan from 127 to 0 (reverse)

    SSD1306_SETCOMPINS,
    0x12,  // COM output scan direction: from COM0 to COM63

    SSD1306_SETCONTRAST,
    0xFF,  // highest contrast

    SSD1306_SETPRECHARGE,
    0xF1,  // set precharge

    SSD1306_SETVCOMDETECT,
    0x00,  // COMvoltage deselect level, using 0x00 seems to give the best
           // brightness levels
    // if it doesn't work, go for 0x040 or 0x20

    SSD1306_DISPLAYALLON_RESUME,  // show ram content on display

    SSD1306_NORMALDISPLAY,  // normal display, not inverse mode

    SSD1306_DISPLAYON};

void oled_init() {
  // outputs
  OLED_DDR |= (1 << OLED_DC_PIN);
  OLED_DDR |= (1 << OLED_RST_PIN);
  _delay_ms(1);  // mandatory high/low/high and delays for proper startup
                 // procedure of display
  OLED_PORT |= (1 << OLED_RST_PIN);
  OLED_PORT &= ~(1 << OLED_RST_PIN);
  _delay_ms(10);
  OLED_PORT |= (1 << OLED_RST_PIN);  // keep reset high
  _delay_ms(10);
  oled_init_commands();
}

void oled_char_small(uint8_t chr) {
  for (int8_t i = 0; i < 5; i++) {  // char bitmap is 5 columns
    oled_send_data(pgm_read_byte(&g_font5x7[chr * 5 + i]));
  }
  oled_send_data(0x00);  // 1 vline of spacing
}

void oled_init_commands() {
  oled_start_command();
  for (uint8_t i = 0; i < sizeof(g_oled_init_commands); i++) {
    oled_send_command(pgm_read_byte(&g_oled_init_commands[i]));
  }
  oled_stop_command();
}

void oled_start_command() {
  OLED_PORT &= ~(1 << OLED_DC_PIN);
  SPI_PORT &= ~(1 << SPI_SS_PIN);
}

void oled_stop_command() {
  SPI_PORT |= (1 << SPI_SS_PIN);
  OLED_PORT |= (1 << OLED_DC_PIN);
}

void oled_send_command(uint8_t command) { spi_communicate(command); }

void oled_start_data() {
  OLED_PORT |= (1 << OLED_DC_PIN);
  SPI_PORT &= ~(1 << SPI_SS_PIN);
}

void oled_stop_data() { SPI_PORT |= (1 << SPI_SS_PIN); }

void oled_send_data(uint8_t data) { spi_communicate(data); }

void oled_clear_screen() {
  oled_stop_scroll();
  oled_set_pos(0, OLED_WIDTH - 1, 0, 7);
  oled_start_data();
  for (uint16_t i = 0; i < (OLED_HEIGHT * OLED_WIDTH / 8); i++) {
    oled_send_data(0x00);
  }
  oled_stop_data();
}

void oled_set_pos(uint8_t col_start, uint8_t col_end, uint8_t page_start,
                  uint8_t page_end) {
  oled_start_command();
  oled_send_command(SSD1306_COLUMNADDR);
  oled_send_command(col_start);  // Column start address (0 = reset)
  oled_send_command(col_end);    // Column end address (127 = reset)
  oled_send_command(SSD1306_PAGEADDR);
  oled_send_command(page_start);  // Page start address (0 = reset)
  oled_send_command(page_end);    // Page end address (7 = reset)
  oled_stop_command();
}

// all pages is start/stop -> 0x00, 0x07
void oled_start_left_scroll(uint8_t start_page, uint8_t stop_page) {
  oled_start_command();
  oled_send_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
  oled_send_command(0X00);
  oled_send_command(start_page);
  oled_send_command(0X00);
  oled_send_command(stop_page);
  oled_send_command(0X00);
  oled_send_command(0XFF);
  oled_send_command(SSD1306_ACTIVATE_SCROLL);
  oled_stop_command();
}

void oled_stop_scroll() {
  oled_start_command();
  oled_send_command(SSD1306_DEACTIVATE_SCROLL);
  oled_stop_command();
}

void oled_clear_graph(void) {
  memset(&g_graph_buffer, 0,
         (uint16_t)(OLED_GRAPH_HEIGHT * OLED_GRAPH_WIDTH / 8));
}

uint8_t oled_calc_graph_ypos(float tempr) {
  uint16_t itempr = tempr * 10;
  if (itempr > OLED_GRAPH_MAX_TEMPR) return 0;
  if (itempr < OLED_GRAPH_MIN_TEMPR) return 63;
  return (OLED_GRAPH_MAX_TEMPR - itempr) * 2;  // 2 y-pixels per 0.1 Celsius
}

void oled_set_startline(uint8_t offset) {
  oled_start_command();
  oled_send_command(SSD1306_SETSTARTLINE | offset);
  oled_stop_command();
}

// formatting the temperature to always have 1 decimal
// using fixed 7 char array
void format_tempr_str(char *str, uint8_t with_dot) {
  if (strlen(str) == 3) {
    if (with_dot) {
      str[3] = str[2];
      str[2] = '.';
      str[4] = '\0';
    }
  } else if (strlen(str) == 2) {
    if (with_dot) {
      str[3] = str[1];
      str[1] = str[0];
      str[0] = '0';
      str[2] = '.';
      str[4] = '\0';
    } else {
      str[2] = str[1];
      str[1] = str[0];
      str[0] = '0';
      str[3] = '\0';
    }
  } else if (strlen(str) == 1) {
    if (with_dot) {
      str[3] = str[0];
      str[0] = '0';
      str[2] = '.';
      str[1] = '0';
      str[4] = '\0';
    } else {
      str[2] = str[0];
      str[0] = '0';
      str[1] = '0';
      str[3] = '\0';
    }
  }
}
