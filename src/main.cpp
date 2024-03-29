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
// 1/9/2023
// Tokyo Andreana


#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <stdint.h>
#include <string.h>
#include <string>
#include <iostream>
#include <cstring>
#include <iomanip>
#include <sstream>


const char* ssid = "xxxx";
const char* password = "xxxx";

ESP8266WebServer server(80);

char* firmware_v = "firmware v 0.2";

//iox defines
const uint8_t iox_read_add = 0x21;
const uint8_t iox_write_add = 0x20;
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
struct pin {
  uint8_t mask;  //mask for io expander
  int port;  //port on io expander (can be 0 or 1 only)
  int pin_number;  //pin number 
  bool onboard;  //If true, IO is on ESP8266; if false, IO is on IO expander
  
};

//struct defines  {mask, port, pin, onboard}
struct pin io_int = {0b0, 0, 14, true};
struct pin sda = {0b0, 0, 4, true};
struct pin scl = {0b0, 0, 2, true};
struct pin htr_1 = {0b0, 0, 13, true};
struct pin htr_2 = {0b0, 0, 12, true};
struct pin fan = {0b0, 0, 5, true};
struct pin ovlight = {0b10000000, 1, 20, false};
struct pin wifi_led_g = {0b00001000, 1, 16, false};
struct pin wifi_led_or = {0b00000100, 1, 15, false};
struct pin wifi_led_r = {0b00000010, 1, 14, false};
struct pin status_led_g = {0b00000001, 1, 13, false};
struct pin status_led_r = {0b10000000, 0, 11, false};
struct pin up_btn = {0b01000000, 1, 19, false};
struct pin dwn_btn = {0b00100000, 1, 18, false};
struct pin ent_btn = {0b00010000, 1, 17, false};
struct pin io_00 = {0b00000001, 0, 4, false};
struct pin io_01 = {0b00000010, 0, 5, false};
struct pin io_02 = {0b00000100, 0, 6, false};
struct pin io_03 = {0b00001000, 0, 7, false};
struct pin io_04 = {0b00010000, 0, 8, false};
struct pin io_05 = {0b00100000, 0, 9, false};
struct pin io_06 = {0b01000000, 0, 10, false};

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

//adc define
int adc_pin = A0;
float adc_offset = -9;  //offset in deg c to copensete for termocuppel inconsisntancy

//pid gain
float Kp = 0.5;  //proportional
float Ki = 0.1;  //integral
float Kd = 0.5;  //derivative
float DT = 0.01; //pid measurement time (seconds)

// reflow cureve defines
// times define the total durration of a given curve segment
// temp is the tempiture setpoint that the oven shold reath durring a given curve segment
struct curve{
  int preheat_time;  // miliseconds
  int preheat_temp;  // deg c
  int soak_time;  // miliseconds
  int soak_temp;  // deg c
  int ramp_to_reflow_time;  // miliseconds
  int ramp_to_reflow_temp;  // deg c
  int reflow_time;  // miliseconds
  int reflow_temp;  // deg c
  int cooling_time;  // miliseconds
  int cooling_temp;  // deg c
  int end_time;  // miliseconds
  
};

// cuve setup {preheat_time, preheat_temp, soak_time, soak_temp, ramp_to_reflow_time, ramp_to_reflow_temp, reflow_time, reflow_temp, cooling_time, cooling_temp, end_time}
//struct curve curve_63_37 = {0, 100, 30000, 150, 120000, 183, 150000, 240, 210000, 25, 327000}; old dont use just for refrance
struct curve curve_63_37 = {0, 100, 90000, 150, 30000, 183, 30000, 240, 30000, 25, 327000};

// heater histores vars
int last_htr_state;
int historesis_time = 1000; // time in mils
unsigned long T_of_last_htr_state = 0;

// 7 seg diplay defines

uint8_t disp_intense = 0xff; // set this number in hex to set 7 segment display brightness

const uint8_t disp_base_add_w = 0b00000000;
const uint8_t disp_base_add_r = 0b00000001;
const uint8_t disp_1_add_w = 0x02;
const uint8_t disp_1_add_r = 0b00000101;
const uint8_t disp_2_add_w = 0x01;
const uint8_t disp_2_add_r = 0b00000011;

// digit write registers
const uint8_t disp_digit_0 = 0b00000001;
const uint8_t disp_digit_1 = 0b00000010;
const uint8_t disp_digit_2 = 0b00000011;
const uint8_t disp_digit_3 = 0b00000100;
const uint8_t disp_digit_4 = 0b00000101;
const uint8_t disp_digit_5 = 0b00000110;
const uint8_t disp_digit_6 = 0b00000111;
const uint8_t disp_digit_7 = 0b00001000;

// config registers
const uint8_t disp_decode_mode = 0b00001001;
const uint8_t disp_global_intensity = 0b00001010;
const uint8_t disp_scan_limit = 0b00001011;
const uint8_t disp_shutdown = 0b00001100;
const uint8_t disp_self_add = 0b00101101;
const uint8_t disp_feature = 0b00001110;
const uint8_t disp_test = 0b00001111;
const uint8_t disp_dig01_int = 0b00010000;
const uint8_t disp_dig23_int = 0b00010001;
const uint8_t disp_dig45_int = 0b00010010;
const uint8_t disp_dig67_int = 0b00010011;

// dig diag registers
const uint8_t disp_dig0_diag = 0b00010100;
const uint8_t disp_dig1_diag = 0b00010101;
const uint8_t disp_dig2_diag = 0b00010110;
const uint8_t disp_dig3_diag = 0b00010111;
const uint8_t disp_dig4_diag = 0b00011000;
const uint8_t disp_dig5_diag = 0b00011001;
const uint8_t disp_dig6_diag = 0b00011010;
const uint8_t disp_dig7_diag = 0b00011011;
const uint8_t disp_keya = 0b00011100;
const uint8_t disp_keyb = 0b00011101;

// shurtdown modes
const uint8_t disp_down_rst = 0x00;
const uint8_t disp_down = 0x80;
const uint8_t disp_up_rst = 0x01;
const uint8_t disp_up = 0x81;

char* prev_disp;

enum MenuOptions {
  CURVE_63_37,
  DISPLAY_IP,
  FIRMWARE_VERSION,
  SET_HOLD
};



// reads current pin state of the io expander outputs
uint8_t read_current_io_state (int port) {
  
  //begin i2c
  Wire.beginTransmission(iox_write_add);

  // selct io port
  if (port == 0){
    Wire.write(iox_output_port_0);
    Wire.endTransmission();
  } else {
    Wire.write(iox_output_port_1);
    Wire.endTransmission();
  }

  // read current state
  Wire.requestFrom(iox_write_add, 1);
  uint8_t readval = 0b0;
 
  readval = Wire.read();
 
  return readval;
}

// io_call function
// This function is used to read or write digital values to onboard or offboard IOs.
// pin_needed: a pin struct that defines the pin being read/written to
// read_write: a read_write enum value that specifies if the function should read or write
// high_low: a high_low enum value that specifies if the function should write a high or low value
int io_call(struct pin pin_needed, enum read_write read_write, enum high_low high_low) {

  

  // Check if the pin is onboard or offboard
  if (pin_needed.onboard == true) {
    // If the pin is onboard, check if we need to read or write
    if (read_write == write) {
      // If we need to write, check if we need to write high or low
      if (high_low == high) { // onborad digital write
        digitalWrite(pin_needed.pin_number, HIGH);
        
      } else if (high_low == low) { // onborad digital write low
        digitalWrite (pin_needed.pin_number, LOW);
        
      } else {
         // If high_low is not high or low, do something here (error handling?)
      }
    } else if (read_write == read) { //onboard digital read
      int read_value = digitalRead(pin_needed.pin_number);
      return(read_value);
    } else {
       // If read_write is not read or write, do something here (error handling?)
    }

  } else if (pin_needed.onboard == false){ //digital write to io expander
    
    if (read_write == write) {  // chek to see if we are writeing the the ioxepander

      // int local varibels
      uint8_t current = read_current_io_state(pin_needed.port); // read and store the current state of the output register of the the given pin
      uint8_t output = 0b00000000;
      uint8_t mask_not = ~pin_needed.mask;
      uint8_t port_reg = 0x00;

      // bitwise logic to ether set high or low a given pin
      if (high_low == high) {
        output = current | pin_needed.mask;
      } else if (high_low == low){
        output = current & mask_not;
      }

      // write to port 0 or 1 based on the given pins's struct 
      if (pin_needed.port == 0) {
        port_reg = iox_output_port_0;
      } else if (pin_needed.port == 1) {
        port_reg = iox_output_port_1;
      }

      Wire.beginTransmission(iox_write_add);  // begin write to iox

      Wire.write(port_reg);

      Wire.write(output);

      Wire.endTransmission();

    } else if (read_write == read) {  // check to see if we are reading from the io expanders input
      
      // init local varibles
      uint8_t readval = 0b00000000;
      uint8_t output_byte = 0b00000000;
      
      // tell the io expander what input port i want to read from 
      Wire.beginTransmission(iox_write_add);
      
      // selcect port based on given pin
      if (pin_needed.port == 0) {
        Wire.write(iox_input_port_0);
      } else if (pin_needed.port == 1) {
        Wire.write(iox_input_port_1);
      }

      Wire.endTransmission();

      // begin reading from the io expander
      Wire.requestFrom(iox_write_add, 1);
      readval = Wire.read();

      // isolate the indivdual bit of the given pin bsed on its mask
      output_byte = readval & pin_needed.mask;

      // return high or low based on wether or not the byte is more than 0
      if (output_byte == 0) {
        return (LOW);
      } else {
        return (HIGH);
      }
      
    }

  } else { // return if input bad
    return 0;
  }
  return 0;

}

// debounced buttion input  (might not work as intened probly need to come back to this)
// butt_pin: struct defines the pin to read
int debounce(struct pin butt_pin) {
  int state = io_call(butt_pin, read, read_mode);
  delay(50);  // debounce sampel time
  if (state == HIGH && io_call(butt_pin, read, read_mode)) {
    return (HIGH);
  } else {
    return (LOW);
  }
}

// read k-type and return celcus
float current_temp() {
  int number_of_readings = 10; // number or readings to avrige for mesurement
  float total_readings = 0;

  for (int i = 0; i < number_of_readings; i++) {
    total_readings = total_readings + analogRead (adc_pin);  // make readings
    delay (10);
  }

  total_readings = total_readings / number_of_readings; // avrige the readings
  
  float voltage = total_readings * 3.3 / 1023.0;// convert to voltage    3.3 is the componat that copenstaes gfor the voltage devider allowing the adc to have a lager tepiture range
  float temp = (voltage / 0.005) + adc_offset;  // covert voltage reading of the adc to celcus
  return (temp);

}

// pid contoll function
// setpoint: target tempiture
float pid(float setpoint) {
  float error = setpoint - current_temp ();
  float integral =  0.0;
  float derivitve = 0.0;
  float lasterror = 0.0;
  int high_range = 100;
  int low_range = -100;

  integral += error * DT;
  derivitve = (error - lasterror) / DT;
  lasterror = error;

  float output1 = Kp * error + Ki * integral + Kd * derivitve;

  output1 = constrain (output1, low_range, high_range);
  float output2 = map(output1, low_range, high_range, -1, 1);

  return (output2);

}

// display blanking function
// just writes 0s to all digit registers on both display drivers resulting in no segments being on
void disp_blank() {


  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_0);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_1);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_2);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_3);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_4);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_5);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_6);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_7);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_0);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_1);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_2);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_3);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_4);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_5);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_6);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_7);
  Wire.write(0x00);
  Wire.endTransmission();
}

// display write function
// to_write: string to be witten to the display
// currently only supports max 16 cahricter input as of right now will add sepping at a later time
char disp_write(char* to_write) {

  disp_blank();

  // defines
  int to_write_len = strlen(to_write);
  char string[to_write_len];
  int decimal_counter = 1;
  int decimal_pos[to_write_len];
  int decimal_shift = 0;
  uint8_t dig_byte[to_write_len];

  for (int j = 0; j < to_write_len; j++) { // write all 0s to the decimal and dig_dyte array
    decimal_pos[j] = 0;
    dig_byte[j] = 0b00000000;
  }
  

  for (int i = 0; i < to_write_len; i++) {  // load input sting to array
    // will load and given cahricter of the input sting in to an array
    // includes logic to bind the approprite designater for spaces decimal points dashes and qwestion marks
    if (to_write[i] == '.') {
      decimal_pos[i - (1 + decimal_shift)] = 1;
      decimal_shift++;
    } else {
      string[i - decimal_shift] = to_write[i];
      //decimal_shift = 0;
    }

  }
 
  for (int k = 0; k < to_write_len; k++) {  // translate form text to bytes
    
    //ive tryed to come up with a better way to do this but ive just given up and am gunna do it the stupid way

    // segment map {0b, DP, A, B, C, D, E, F, G}
    //   a
    //  ___
    // f|g|b
    //  ___
    // e|d|c
    //  ___DP
    
    uint8_t current_char = 0b0;
    
    if (string[k] == '0') {
      current_char = 0b01111110;
    } else if (string[k] == '1') {
      current_char = 0b00110000;
    } else if (string[k] == '2') {
      current_char = 0b01101101;
    } else if (string[k] == '3') {
      current_char = 0b01111001;
    } else if (string[k] == '4') {
      current_char = 0b00110011;
    } else if (string[k] == '5') {
      current_char = 0b01011011;
    } else if (string[k] == '6') {
      current_char = 0b01011111;
    } else if (string[k] == '7') {
      current_char = 0b01110000;
    } else if (string[k] == '8') {
      current_char = 0b01111111;
    } else if (string[k] == '9') {
      current_char = 0b01111011;
    } else if (string[k] == '-') {
      current_char = 0b00000001;
    } else if (string[k] == 'a') {
      current_char = 0b01111101;
    } else if (string[k] == 'b') {
      current_char = 0b00011111;
    } else if (string[k] == 'c') {
      current_char = 0b00001101;
    } else if (string[k] == 'd') {
      current_char = 0b00111101;
    } else if (string[k] == 'e') {
      current_char = 0b01001111;
    } else if (string[k] == 'f') {
      current_char = 0b01000111;
    } else if (string[k] == 'g') {
      current_char = 0b01011110;
    } else if (string[k] == 'h') {
      current_char = 0b00010111;
    } else if (string[k] == 'i') {
      current_char = 0b00000110;
    } else if (string[k] == 'j') {
      current_char = 0b01011000;
    } else if (string[k] == 'k') {
      current_char = 0b01010111;
    } else if (string[k] == 'l') {
      current_char = 0b00001110;
    } else if (string[k] == 'm') {
      current_char = 0b01010101;
    } else if (string[k] == 'n') {
      current_char = 0b00010101;
    } else if (string[k] == 'o') {
      current_char = 0b00011101;
    } else if (string[k] == 'p') {
      current_char = 0b01100111;
    } else if (string[k] == 'q') {
      current_char = 0b01110011;
    } else if (string[k] == 'r') {
      current_char = 0b00000101;
    } else if (string[k] == 's') {
      current_char = 0b01101101;
    } else if (string[k] == 't') {
      current_char = 0b00001111;
    } else if (string[k] == 'u') {
      current_char = 0b00011100;
    } else if (string[k] == 'v') {
      current_char = 0b00101010;
    } else if (string[k] == 'w') {
      current_char = 0b00101011;
    } else if (string[k] == 'x') {
      current_char = 0b00010100;
    } else if (string[k] == 'y') {
      current_char = 0b00111011;
    } else if (string[k] == 'z') {
      current_char = 0b01101100;
    } else if (string[k] == ' ') {
      current_char = 0b00000000;
    } //else if (string[k] == '?') {
      //current_char = 0b11110010;
    //}
   


    if (decimal_pos[k] == 1){  // attach decimal point if needed
      dig_byte[k] = current_char | 0b10000000;
    } else {
      dig_byte[k] = current_char;
    }  
    
  }
 
  for (int l = 0; l < to_write_len; l++) {  //write to display 
    if (to_write_len <= (15 + decimal_counter)) {
      if (l == 0) {
        Wire.beginTransmission(disp_1_add_w);  // write cahr 0 to display
        Wire.write(disp_digit_0);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 1) {
        Wire.beginTransmission(disp_1_add_w);  // write cahr 1 to display
        Wire.write(disp_digit_1);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 2) {
        Wire.beginTransmission(disp_1_add_w);  // write cahr 2 to display
        Wire.write(disp_digit_2);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 3) {
        Wire.beginTransmission(disp_1_add_w);  // write cahr 3 to display
        Wire.write(disp_digit_3);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 4) {
        Wire.beginTransmission(disp_2_add_w);  // write cahr 4 to display
        Wire.write(disp_digit_0);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 5) {
        Wire.beginTransmission(disp_2_add_w);  // write cahr 5 to display
        Wire.write(disp_digit_1);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 6) {
        Wire.beginTransmission(disp_2_add_w);  // write cahr 6 to display
        Wire.write(disp_digit_2);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 7) {
        Wire.beginTransmission(disp_2_add_w);  // write cahr 7 to display
        Wire.write(disp_digit_3);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 8) {
        Wire.beginTransmission(disp_1_add_w);  // write cahr 8 to display
        Wire.write(disp_digit_4);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 9) {
        Wire.beginTransmission(disp_1_add_w);  // write cahr 9 to display
        Wire.write(disp_digit_5);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 10) {
        Wire.beginTransmission(disp_1_add_w);  // write cahr 10 to display
        Wire.write(disp_digit_6);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 11) {
        Wire.beginTransmission(disp_1_add_w);  // write cahr 11 to display
        Wire.write(disp_digit_7);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 12) {
        Wire.beginTransmission(disp_2_add_w);  // write cahr 12 to display
        Wire.write(disp_digit_4);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 13) {
        Wire.beginTransmission(disp_2_add_w);  // write cahr 13 to display
        Wire.write(disp_digit_5);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 14) {
        Wire.beginTransmission(disp_2_add_w);  // write cahr 14 to display
        Wire.write(disp_digit_6);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else if (l == 15) {
        Wire.beginTransmission(disp_2_add_w);  // write cahr 15 to display
        Wire.write(disp_digit_7);
        Wire.write(dig_byte[l]);
        Wire.endTransmission();
      } else {
        return('overflow error');                     //replace these with overflows with appropreate handeling logic
      }
    } else {
      return('overflow error');
    }
  }
  return 0;
}

// display 2 line write
// just takes 2 inputs and put the appropreate number of spaces inbetween
void disp_write_2_line(float line_1, float line_2) {
  std::ostringstream output;
  output << std::left << std::setw(9) << std::fixed << std::setprecision(2) << line_1;
  output << std::fixed << std::setprecision(2) << line_2;
  std::string message = output.str();
  char* c_message = new char[message.length() + 1];
  strcpy(c_message, message.c_str());
  disp_write(c_message);
  delete[] c_message;
}

// heater control
// input: takes a float input from -1 to 1 and based on that turns on or off the heaters
void heater_control (float input) {
  if ((millis() - (T_of_last_htr_state)) < historesis_time) {
    
  } else {
    if (input > 0 && input < 0.5) { // htr state 1
      io_call (htr_1, write, high);
      io_call (htr_2, write, low);
      last_htr_state = 1;
    } else if (input > 0.5) {  // htr state 2
      io_call (htr_1, write, high);
      io_call (htr_2, write, high);
      last_htr_state = 2;
    } else if (input < 0) {  // htr state 0
      io_call (htr_1, write, low);
      io_call (htr_2, write, low);
      last_htr_state = 0;
    }
    T_of_last_htr_state = millis();
  }
  

}

// reflow control function
// curve_needed: struct of reflow cure to follow
int reflow_control (struct curve curve_needed) {

  // turn on oven fan and light aswell as cahange the status led
  io_call(fan, write, high);
  io_call(ovlight, write, high);
  io_call(status_led_g, write, low);
  io_call(status_led_r, write, high);

  Serial.println("refow loop enterd");

  // defines
  unsigned long stop_time;
  bool exit_state = false;
  //unsigned long cooling_time = 0;
  bool exit_trigger = false;
  //bool reflow_complete = false;
  //unsigned long reflow_stop_time = 0;
  unsigned long pre_heat_time = 0;
  bool pre_heat_complete = false;
  unsigned long soak_time = 0;
  bool soak_compltete = false;
  unsigned long ramp_time = 0;
  bool ramp_complete = false;
  unsigned long reflow_time = 0;
  bool reflow_complete = false;
  unsigned long cooling_time = 0;
  bool cooling_compltete = false; //idk if i acc need this

  // oven pre heat and wait for laoding

  /*

  delay (500);

  while (debounce(ent_btn) == HIGH) {
    Serial.println(current_temp());
    if (current_temp() < curve_needed.preheat_temp) {
      heater_control(pid(curve_needed.preheat_temp));
      disp_write("please wait");
      delay(1000);
      disp_write("prehaeting");
      delay(1000);
    } else {
      heater_control(pid(curve_needed.preheat_temp));
      disp_write("preheat complete");
      delay(1000);
      disp_write("hold enter");
      delay (1000);
      disp_write("when oven loaded");
      delay (1000);
    }
  }


  // blink lights between stages
  io_call(ovlight, write, high);
  delay(200);
  io_call(ovlight, write, low);
  delay(200);
  io_call(ovlight, write, high);
  delay(200);
  io_call(ovlight, write, low);
  delay(200);
  io_call(ovlight, write, high);
  delay(200);
  io_call(ovlight, write, low);
  delay(200);
  io_call(ovlight, write, high);

  */
  
  
  // start timer
  unsigned long start_time;
  start_time = millis();
  
  // control loop
  while (exit_trigger == false) {


    
    if (pre_heat_complete == false) {  // preheat stage
      heater_control(pid(curve_needed.preheat_temp));
      if ((millis() - start_time) < 3000){
        disp_write("pre-heat stage");
      } else {
        disp_write_2_line(current_temp(), curve_needed.preheat_temp);
      }
      if ((curve_needed.preheat_temp < current_temp()) && (pre_heat_time == 0)) {
        pre_heat_time = millis();
      }
      if (((millis() - pre_heat_time) > curve_needed.preheat_time) && (pre_heat_time != 0)){
        pre_heat_complete = true;
      }
    } else if (soak_compltete == false && pre_heat_complete == true) {  // soak stage
      heater_control(pid(curve_needed.soak_temp));
      if ((millis() - (pre_heat_time + curve_needed.preheat_time)) < 3000){
        disp_write("soak stage");
        if ((millis() - (pre_heat_time + curve_needed.preheat_time)) < 1000) {
          io_call(ovlight, write, low);
        } else if ((millis() - (pre_heat_time + curve_needed.preheat_time)) < 2000) {
          io_call(ovlight, write, high);
        }
      } else {
        disp_write_2_line(current_temp(), curve_needed.soak_temp);
      }
      if ((curve_needed.soak_temp > current_temp()) && (soak_time == 0)){
        soak_time = millis();
      }
      if (((millis() - soak_time) > curve_needed.soak_time) && ( soak_time != 0)) {
        soak_compltete = true;
      }
    } else if (pre_heat_complete == true && soak_compltete == true && ramp_complete == false) {  // ramp to reflow stage
      heater_control(pid(curve_needed.ramp_to_reflow_temp));
      if ((millis() - (soak_time + curve_needed.soak_time)) < 3000){
        disp_write("ramp stage");
        if ((millis() - (soak_time + curve_needed.soak_time)) < 1000) {
          io_call(ovlight, write, low);
        } else if ((millis() - (soak_time + curve_needed.soak_time)) < 2000) {
          io_call(ovlight, write, high);
        }
      } else {
        disp_write_2_line(current_temp(), curve_needed.ramp_to_reflow_temp);
      }
      if ((curve_needed.ramp_to_reflow_temp > current_temp()) && (ramp_time == 0)) {
        ramp_time = millis();
      }
      if (((millis() - ramp_time) > curve_needed.ramp_to_reflow_time) && (ramp_time != 0)){
        ramp_complete = true;
      }
    } else if (pre_heat_complete == true && soak_compltete == true && ramp_complete && reflow_complete == false) {  // reflow stage
      heater_control(pid(curve_needed.reflow_temp));
      Serial.println("reflow stage");
      
      if ((millis() - (ramp_time + curve_needed.ramp_to_reflow_time)) < 3000){
        disp_write("reflow stage");
        if ((millis() - (ramp_time + curve_needed.ramp_to_reflow_time)) < 1000) {
          io_call(ovlight, write, low);
        } else if ((millis() - (ramp_time + curve_needed.ramp_to_reflow_time)) < 2000) {
          io_call(ovlight, write, high);
        }
      } else {
        disp_write_2_line(current_temp(), curve_needed.reflow_temp);
      }
      if ((current_temp() > curve_needed.reflow_temp) && (reflow_time == 0)) {
        reflow_time = millis();
      }
      if (((millis() - reflow_time) > curve_needed.reflow_time) && (reflow_time != 0)) {
        reflow_complete = true;
      }
    } else if (pre_heat_complete == true && soak_compltete == true && ramp_complete == true && reflow_complete == true && cooling_compltete == false) {  // cooling stage
      heater_control(pid(curve_needed.cooling_temp));
      Serial.println("cooling stage?");
      if (cooling_time = 0) {
        cooling_time = millis();
      }
      
      if ((millis() - (reflow_time + curve_needed.reflow_time)) < 3000){
        disp_write("cooling stage");
        if ((millis() - (reflow_time + curve_needed.reflow_time)) < 1000) {
          io_call(ovlight, write, low);
        } else if ((millis() - (reflow_time + curve_needed.reflow_time)) < 2000) {
          io_call(ovlight, write, high);
        }
      } else {
        disp_write_2_line(current_temp(), curve_needed.cooling_temp);
      }
      
    } else if (pre_heat_complete == true && soak_compltete == true && ramp_complete == true && reflow_complete == true && cooling_compltete == true) {
      exit_trigger = true;
    } else {

    }

    if (io_call(ent_btn, read, read_mode) == LOW) { // exit reflow loop early if 'ent' buttion is held for 3 seconds
      if (!exit_state) {
        stop_time = millis();
        exit_state = true;
      } else {
        if ((millis() - stop_time) >= 3000) {
          //return to defult state
          io_call(status_led_g, write, low);
          delay(100);
          io_call(status_led_g, write, high);
          delay(100);
          io_call(status_led_g, write, low);
          delay(100);
          io_call(status_led_g, write, high);
          delay(100);
          io_call(status_led_g, write, low);
          delay(100);
          io_call(status_led_g, write, high);
          delay(100);
          io_call(fan, write, low);
          io_call(ovlight, write, low);
          io_call(status_led_g, write, high);
          io_call(status_led_r, write, low);
          io_call(htr_1, write, low);
          io_call(htr_2, write, low);
          delay (1000);
          return 0;          
        }
      }

      
    } else {
      exit_state = false;
    }

    delay(100); // loop will run evry 1/10th second
    
  }
  
  // turn off oven fan
  io_call(fan, write, low);
  io_call(ovlight, write, low);
  io_call(status_led_g, write, high);
  io_call(status_led_r, write, low);
  io_call(htr_1, write, low);
  io_call(htr_2, write, low);
  
  return 0;
    
}

// oven preheat/hold
int temp_hold () {

  int setpoint = 0;
  unsigned long start_time;

  disp_write("enter set-point");

  delay(3000);

  while (debounce(ent_btn) == HIGH) {
    if (debounce(up_btn) == LOW){
      setpoint++;
    } else if (debounce(dwn_btn) == LOW) {
      setpoint--;
    }
    
    Serial.println(setpoint);

    disp_write_2_line(setpoint, setpoint);
  }

  start_time = millis();
  io_call(status_led_g, write, low);
  io_call(status_led_r, write, high);
  io_call(ovlight, write, high);

  io_call(fan, write, high);

  disp_write("enter to exit");
  delay(2000);
  disp_write("begin pre-heat");
  delay(3000);

  while (debounce(ent_btn) == HIGH) {
    if (current_temp() > (setpoint + 10) && current_temp() < (setpoint - 10)) {
      disp_write("done heating");
      io_call(ovlight, write, high);
      delay (1000);
      io_call(ovlight, write, low);
    } else {
      heater_control(pid(setpoint));
      disp_write_2_line(current_temp(), setpoint);
    }

  }

  delay(1000);
  io_call(status_led_r, write, low);
  io_call(ovlight, write, low);
  io_call(fan, write, low);
return 0;
}



void setup() {

  // pin modes for onboard io
  pinMode(htr_1.pin_number, OUTPUT);  
  pinMode(htr_2.pin_number, OUTPUT);  
  pinMode(fan.pin_number, OUTPUT);
  pinMode(io_int.pin_number, INPUT);

  // start serial
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("serial started");

  // start i2c
  Wire.begin(sda.pin_number, scl.pin_number);
  Wire.setClock(400000); //
  Serial.println("i2c started");
  
  // io epander init
  
  
  //set pinmodes 
  // pinmode 0=output 1=input match to pin mask
  Wire.beginTransmission(iox_write_add);
  Wire.write(iox_config_port_0);  //port 0
  Wire.write(0b00000000);                  //will need to change this to set gpio pinmodes
  Wire.endTransmission();
  Wire.beginTransmission(iox_write_add);
  Wire.write(iox_config_port_1); //port 1
  Wire.write(0b01110000);
  Wire.endTransmission();
  Serial.println("io expander pin modes set");

  
  // set all io expander outputs low
  Wire.beginTransmission(iox_write_add);
  Wire.write(iox_output_port_0);  //port 0
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(iox_write_add);
  Wire.write(iox_output_port_1);  //port 1
  Wire.write(0b00000000);
  Wire.endTransmission();
  Serial.println("io expander all pins set to low");
  
  // fancy boot up seqwince 
  io_call(status_led_g, write, high);
  delay(100);
  io_call(status_led_r, write, high);
  delay(100);
  io_call(wifi_led_r, write, high);
  delay(100);
  io_call(wifi_led_or, write, high);
  delay(100);
  io_call(wifi_led_g, write, high);
  delay(100);
  io_call(status_led_g, write, low);
  delay(100);
  io_call(status_led_r, write, low);
  delay(100);
  io_call(wifi_led_r, write, low);
  delay(100);
  io_call(wifi_led_or, write, low);
  delay(100);
  io_call(wifi_led_g, write, low);
  delay(100);
  io_call(status_led_g, write, high);
  delay(100);
  io_call(status_led_r, write, high);
  delay(100);
  io_call(wifi_led_r, write, high);
  delay(100);
  io_call(wifi_led_or, write, high);
  delay(100);
  io_call(wifi_led_g, write, high);
  delay(100);
  io_call(status_led_g, write, low);
  delay(100);
  io_call(status_led_r, write, low);
  delay(100);
  io_call(wifi_led_r, write, low);
  delay(100);
  io_call(wifi_led_or, write, low);
  delay(100);
  io_call(wifi_led_g, write, low);


/*
  
  // start wifi
  io_call(wifi_led_r, write, high);
  Serial.println("WIFI connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    io_call(wifi_led_or, write, high);
    Serial.print('.');
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    io_call(wifi_led_r, write, low);
    io_call(wifi_led_or, write, low);
    io_call(wifi_led_g, write, high);
    Serial.println(' ');
    Serial.println("WIFI connected");
    Serial.print("IP: ");
    Serial.print(WiFi.localIP());
    Serial.println(' ');
  } else if (WiFi.status() == WL_CONNECT_FAILED) {
    io_call(wifi_led_or, write, low);
    io_call(wifi_led_r, write, high);
    Serial.println(' ');
    Serial.println("WIFI connection failed");
  } else if (WiFi.status() == WL_NO_SSID_AVAIL) {
    io_call(wifi_led_or, write, low);
    io_call(wifi_led_r, write, high);
    Serial.println(' ');
    Serial.println("SSID not availabel");
  } else if (WiFi.status() == WL_CONNECTION_LOST) {
    io_call(wifi_led_or, write, low);
    io_call(wifi_led_r, write, high);
    Serial.println(' ');
    Serial.println("WIFI connection lost");
  } 
  
*/

  // begin display init
  Wire.beginTransmission(disp_base_add_w);
  Wire.write(disp_shutdown);
  Wire.write(disp_up_rst);
  Wire.endTransmission();

  // display self address command
  Wire.beginTransmission(disp_base_add_w);
  Wire.write(disp_self_add);                  
  Wire.write(0b00000001);
  Wire.endTransmission();
  Serial.println("display self addressed");

  // pull display 1 out of shutdown mode
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_shutdown);
  Wire.write(disp_up);
  Wire.endTransmission();

  // pull display 2 out of shutdown mode
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_shutdown);
  Wire.write(disp_up);
  Wire.endTransmission();
  Serial.println("display pulled out of shut down");
  
  //diplay decode mode command
  Wire.beginTransmission(disp_1_add_w);  //display dvr 1
  Wire.write(disp_decode_mode);
  Wire.write(0b00000000);  // decode mode raw
  Wire.endTransmission();

  Wire.beginTransmission(disp_2_add_w);  //display dvr 2
  Wire.write(disp_decode_mode);
  Wire.write(0b00000000);  // decode mode raw
  Wire.endTransmission();
  Serial.println("display decode mode set to raw");

  // display scan limit setup
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_scan_limit);
  Wire.write(0b00000111);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_scan_limit);
  Wire.write(0b00000111);
  Wire.endTransmission();
  Serial.println("display scan limit set");

  // disply intesity setup
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_global_intensity);
  Wire.write(disp_intense);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_global_intensity);
  Wire.write(disp_intense);
  Wire.endTransmission();
  Serial.println("display intensity set");

  // display self test
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_test);
  Wire.write(0b10000000);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_test);
  Wire.write(0b10000000);
  Wire.endTransmission();
  delay(500);
  Wire.beginTransmission(disp_1_add_w);  //end test
  Wire.write(disp_test);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_test);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Serial.println("display self test complete");


}

void loop() {

  io_call(status_led_g, write, high);
  
  static MenuOptions menuOption = CURVE_63_37;

  // continuously update the display
  switch (menuOption) {
    case CURVE_63_37:
      disp_write("63 37 curve");
      break;
    case DISPLAY_IP:
      disp_write("display ip ?");
      break;
    case FIRMWARE_VERSION:
      disp_write(firmware_v);
      break;
    case SET_HOLD:
      disp_write("set-point hold");
      break;
    default:
      disp_write("error");
      break;
  }

  // check for button press
  if (debounce(up_btn) == LOW) {
    menuOption = (menuOption == CURVE_63_37) ? DISPLAY_IP : (menuOption == DISPLAY_IP) ? FIRMWARE_VERSION : (menuOption == FIRMWARE_VERSION) ? SET_HOLD : CURVE_63_37;
  } else if (debounce(ent_btn) == LOW) {
    switch (menuOption) {
      case CURVE_63_37:
        reflow_control(curve_63_37);
        break;
      case DISPLAY_IP:
        //IPAddress ip = WiFi.localIP();
        //String ip_str = ip.toString();
        //disp_write((char*)ip_str.c_str());
        disp_write("192.168.202.202");
        delay (3000);
        break;
      case FIRMWARE_VERSION:
        disp_write(firmware_v);
        break;
      case SET_HOLD:
        temp_hold();
        break;
      default:
        disp_write("error");
        break;
    }
  } else if (debounce(dwn_btn) == LOW) {
    menuOption = (menuOption == DISPLAY_IP) ? CURVE_63_37 : (menuOption == FIRMWARE_VERSION) ? DISPLAY_IP : (menuOption == DISPLAY_IP) ? SET_HOLD : FIRMWARE_VERSION;
  }


}


