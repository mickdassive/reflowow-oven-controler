//{reflow oven firmware}
//Copyright (C) {2023}  {mickmake}
//
//This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with this program.  If not, see <http://www.gnu.org/licenses/>.
// reflow oven firmware
// v0.1
// 23/06/2025
// Tokyo Andreana

#ifndef display_h
#define display_h



#include <Arduino.h>



// 7 seg diplay defines
const uint8_t disp_base_add_w = 0x00;
const uint8_t disp_1_add_w = 0x02;
const uint8_t disp_2_add_w = 0x01;

// digit write registers
const uint8_t disp_digit_0 = 0x01;
const uint8_t disp_digit_1 = 0x02;
const uint8_t disp_digit_2 = 0x03;
const uint8_t disp_digit_3 = 0x04;
const uint8_t disp_digit_4 = 0x05;
const uint8_t disp_digit_5 = 0x06;
const uint8_t disp_digit_6 = 0x07;
const uint8_t disp_digit_7 = 0x08;

// config registers
const uint8_t disp_decode_mode = 0x09;
const uint8_t disp_global_intensity = 0x0A;
const uint8_t disp_scan_limit = 0x0B;
const uint8_t disp_shutdown = 0x0C;
const uint8_t disp_self_add = 0x2D;
const uint8_t disp_feature = 0x0E;
const uint8_t disp_test = 0x0F;
const uint8_t disp_dig01_int = 0x10;
const uint8_t disp_dig23_int = 0x11;
const uint8_t disp_dig45_int = 0x12;
const uint8_t disp_dig67_int = 0x13;

// dig diag registers
const uint8_t disp_dig0_diag = 0x14;
const uint8_t disp_dig1_diag = 0x15;
const uint8_t disp_dig2_diag = 0x16;
const uint8_t disp_dig3_diag = 0x17;
const uint8_t disp_dig4_diag = 0x18;
const uint8_t disp_dig5_diag = 0x19;
const uint8_t disp_dig6_diag = 0x1A;
const uint8_t disp_dig7_diag = 0x1B;
const uint8_t disp_keya = 0x1C;
const uint8_t disp_keyb = 0x1D;

// shurtdown modes
const uint8_t disp_down_rst = 0x00;
const uint8_t disp_down = 0x80;
const uint8_t disp_up_rst = 0x01;
const uint8_t disp_up = 0x81;

extern char* prev_disp;

// display alfbet
struct disp_alfbet {
  char simbol;
  uint8_t value;
};

// segment map {DP, A, B, C, D, E, F, G}
//   a
//  ___
// f|g|b
//  ___
// e|d|c
//  ___DP

static struct disp_alfbet disp_7seg_alfbet_0 = {'0', 0b01111110};
static struct disp_alfbet disp_7seg_alfbet_1 = {'1', 0b00110000};
static struct disp_alfbet disp_7seg_alfbet_2 = {'2', 0b01101101};
static struct disp_alfbet disp_7seg_alfbet_3 = {'3', 0b01111001};
static struct disp_alfbet disp_7seg_alfbet_4 = {'4', 0b00110011};
static struct disp_alfbet disp_7seg_alfbet_5 = {'5', 0b01011011};
static struct disp_alfbet disp_7seg_alfbet_6 = {'6', 0b01011111};
static struct disp_alfbet disp_7seg_alfbet_7 = {'7', 0b01110000};
static struct disp_alfbet disp_7seg_alfbet_8 = {'8', 0b01111111};
static struct disp_alfbet disp_7seg_alfbet_9 = {'9', 0b01111011};
static struct disp_alfbet disp_7seg_alfbet_dash = {'-', 0b00000001};
static struct disp_alfbet disp_7seg_alfbet_space = {' ', 0b00000000};
static struct disp_alfbet disp_7seg_alfbet_a = {'a', 0b01111101};
static struct disp_alfbet disp_7seg_alfbet_b = {'b', 0b00011111};
static struct disp_alfbet disp_7seg_alfbet_c = {'c', 0b00001101};
static struct disp_alfbet disp_7seg_alfbet_d = {'d', 0b00111101};
static struct disp_alfbet disp_7seg_alfbet_e = {'e', 0b01001111};
static struct disp_alfbet disp_7seg_alfbet_f = {'f', 0b01000111};
static struct disp_alfbet disp_7seg_alfbet_g = {'g', 0b01011110};
static struct disp_alfbet disp_7seg_alfbet_h = {'h', 0b00010111};
static struct disp_alfbet disp_7seg_alfbet_i = {'i', 0b00000110};
static struct disp_alfbet disp_7seg_alfbet_j = {'j', 0b01011000};
static struct disp_alfbet disp_7seg_alfbet_k = {'k', 0b01010111};
static struct disp_alfbet disp_7seg_alfbet_l = {'l', 0b00001110};
static struct disp_alfbet disp_7seg_alfbet_m = {'m', 0b01010101};
static struct disp_alfbet disp_7seg_alfbet_n = {'n', 0b00110101};
static struct disp_alfbet disp_7seg_alfbet_o = {'o', 0b00111110};
static struct disp_alfbet disp_7seg_alfbet_p = {'p', 0b01110111};
static struct disp_alfbet disp_7seg_alfbet_q = {'q', 0b01110011};
static struct disp_alfbet disp_7seg_alfbet_r = {'r', 0b00110111};
static struct disp_alfbet disp_7seg_alfbet_s = {'s', 0b01011011};
static struct disp_alfbet disp_7seg_alfbet_t = {'t', 0b01110101};
static struct disp_alfbet disp_7seg_alfbet_u = {'u', 0b00111100};
static struct disp_alfbet disp_7seg_alfbet_v = {'v', 0b00111000};
static struct disp_alfbet disp_7seg_alfbet_w = {'w', 0b00111001};
static struct disp_alfbet disp_7seg_alfbet_x = {'x', 0b01110101};
static struct disp_alfbet disp_7seg_alfbet_y = {'y', 0b01111101};
static struct disp_alfbet disp_7seg_alfbet_z = {'z', 0b01001101};

/**
 * @brief Lookup table for 7-segment display character representations.
 *
 * This array contains pointers to constant structures of type disp_alfbet,
 * each representing the segment encoding for a specific character on a 7-segment display.
 * The table includes digits (0-9), dash, space, and uppercase alphabetic characters (A-Z).
 *
 * The order of elements is as follows:
 *   - Digits: 0-9
 *   - Special characters: dash, space
 *   - letters: a-z (lowercase, no uppercase available)
 *
 * Each entry (e.g., &disp_7seg_alfbet_a) points to the segment pattern for the corresponding character.
 * This table can be used to map alphanumeric characters to their 7-segment display encodings
 * for rendering text or numbers on a 7-segment display module.
 *
 * @note Ensure that the disp_alfbet structures for each character are defined elsewhere in the codebase.
 */
[[maybe_unused]] static struct disp_alfbet* disp_7seg_alfbet[] = {
    &disp_7seg_alfbet_0,
    &disp_7seg_alfbet_1,
    &disp_7seg_alfbet_2,
    &disp_7seg_alfbet_3,
    &disp_7seg_alfbet_4,
    &disp_7seg_alfbet_5,
    &disp_7seg_alfbet_6,
    &disp_7seg_alfbet_7,
    &disp_7seg_alfbet_8,
    &disp_7seg_alfbet_9,
    &disp_7seg_alfbet_dash,
    &disp_7seg_alfbet_space,
    &disp_7seg_alfbet_a,
    &disp_7seg_alfbet_b,
    &disp_7seg_alfbet_c,
    &disp_7seg_alfbet_d,
    &disp_7seg_alfbet_e,
    &disp_7seg_alfbet_f,
    &disp_7seg_alfbet_g,
    &disp_7seg_alfbet_h,
    &disp_7seg_alfbet_i,
    &disp_7seg_alfbet_j,
    &disp_7seg_alfbet_k,
    &disp_7seg_alfbet_l,
    &disp_7seg_alfbet_m,
    &disp_7seg_alfbet_n,
    &disp_7seg_alfbet_o,
    &disp_7seg_alfbet_p,
    &disp_7seg_alfbet_q,
    &disp_7seg_alfbet_r,
    &disp_7seg_alfbet_s,
    &disp_7seg_alfbet_t,
    &disp_7seg_alfbet_u,
    &disp_7seg_alfbet_v,
    &disp_7seg_alfbet_w,
    &disp_7seg_alfbet_x,
    &disp_7seg_alfbet_y,
    &disp_7seg_alfbet_z
};

// function defines
void disp_blank();
void disp_write(const char* to_write);
void disp_init();
void disp_set_brightness(uint8_t brightness);
void disp_write_2_line(float line_1, float line_2);




#endif // display_h