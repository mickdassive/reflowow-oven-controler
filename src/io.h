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

#ifndef io_h
#define io_h

#include <Arduino.h>

//iox defines
const uint8_t iox_add = 0x20;
const uint8_t iox_input_port_0 = 0x00;
const uint8_t iox_input_port_1 = 0x01;
const uint8_t iox_output_port_0 = 0x02;
const uint8_t iox_output_port_1 = 0x03;
const uint8_t iox_pol_inv_port_0 = 0x04;
const uint8_t iox_pol_inv_port_1 = 0x05;
const uint8_t iox_config_port_0 = 0x06;
const uint8_t iox_config_port_1 = 0x07;
const uint8_t iox_drive_strength_register_00 = 0x40;
const uint8_t iox_drive_strenght_register_01 = 0x41;
const uint8_t iox_drive_strenght_register_10 = 0x42;
const uint8_t iox_drive_strenght_register_11 = 0x43;
const uint8_t iox_in_latch_register_0 = 0x44;
const uint8_t iox_in_ltach_register_1 =0x45;
const uint8_t iox_pull_up_down_en_register_0 = 0x46;
const uint8_t iox_pull_up_down_en_register_1 = 0x47;
const uint8_t iox_pull_up_down_sel_register_0 = 0x48;
const uint8_t iox_pull_up_down_sel_register_1 = 0x49;
const uint8_t iox_int_mask_register_0 = 0x4a;
const uint8_t iox_int_mask_register_1 = 0x4b;
const uint8_t iox_int_stat_register_0 = 0x4c;
const uint8_t iox_int_stat_register_1 = 0x4d;
const uint8_t iox_out_port_config_register = 0x4f;

//struct setup for pin cals

//io_call struct setup
enum in_out {
  in,       //input
  out,      //output
  bi_dir,   //bi-directional pin only used for i2c
  intr,     //used for software interupts only works with onboard pins
  empty_pin //empty for empty pin object
};

struct pin {
  uint8_t mask;  //mask for io expander
  int port;  //port on io expander (can be 0 or 1 only)
  int pin_number;  //pin number 
  bool onboard;  //If true, IO is on ESP8266; if false, IO is on IO expander
  enum in_out pin_mode;       //defines pinmode see enum above for deatails
};

//struct defines  {mask, port, pin, onboard, pinmode}
static struct pin io_int = {0, 0, 14, true, intr};
static struct pin sda = {0, 0, 4, true, bi_dir};
static struct pin scl = {0, 0, 2, true, bi_dir};
static struct pin htr_1 = {0, 0, 13, true, out};
static struct pin htr_2 = {0, 0, 12, true, out};
static struct pin fan = {0, 0, 5, true, out};
static struct pin ovlight = {0b10000000, 1, 20, false, out};
static struct pin wifi_led_g = {0b00001000, 1, 16, false, out};
static struct pin wifi_led_or = {0b00000100, 1, 15, false, out};
static struct pin wifi_led_r = {0b00000010, 1, 14, false, out};
static struct pin status_led_g = {0b00000001, 1, 13, false, out};
static struct pin status_led_r = {0b10000000, 0, 11, false, out};
static struct pin up_btn = {0b01000000, 1, 19, false, intr};
static struct pin dwn_btn = {0b00100000, 1, 18, false, intr};
static struct pin ent_btn = {0b00010000, 1, 17, false, intr};
static struct pin io_00 = {0b00000001, 0, 4, false, out};
static struct pin io_01 = {0b00000010, 0, 5, false, out};
static struct pin io_02 = {0b00000100, 0, 6, false, out};
static struct pin io_03 = {0b00001000, 0, 7, false, out};
static struct pin io_04 = {0b00010000, 0, 8, false, out};
static struct pin io_05 = {0b00100000, 0, 9, false, out};
static struct pin io_06 = {0b01000000, 0, 10, false, out};

// read write enum define for io_call function
enum read_write {
  read,
  write
};

// io_call high low input enum
enum high_low {
  high,
  low,
  read_mode
};

[[maybe_unused]] static struct pin* pin_names[] = {
  &io_int,
  &sda,
  &scl,
  &htr_1,
  &htr_2,
  &fan,
  &ovlight,
  &wifi_led_g,
  &wifi_led_or,
  &wifi_led_r,
  &status_led_g,
  &status_led_r,
  &up_btn,
  &dwn_btn,
  &ent_btn,
  &io_00,
  &io_01,
  &io_02,
  &io_03,
  &io_04,
  &io_05,
  &io_06
};

//adc defines
const int adc_pin = A0;
const float adc_offset = -9;  //offset in deg c to copensete for termocuppel inconsisntancy

//function defines
int io_call(struct pin pin_needed, enum read_write rw, enum high_low hl);
void io_gpio_init();
int io_debounce(struct pin butt_pin);
void io_heater_control(float input);
void io_set_heater_historesis (int time);
float io_read_current_temp();





#endif // io_h