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


const char* ssid = "xxxx";
const char* password = "xxxx";

ESP8266WebServer server(80);

//iox defines
const uint8_t iox_read_add = 0b01000010;
const uint8_t iox_write_add = 0b01000011;
const uint8_t iox_read_port_0 = 0b00000000;
const uint8_t iox_read_port_1 = 0b00000001;
const uint8_t iox_write_port_0 = 0b00000010;
const uint8_t iox_write_port_1 = 0b00000011;
const uint8_t iox_pol_inv_port_0 = 0b00000100;
const uint8_t iox_pol_inv_port_1 = 0b00000101;
const uint8_t iox_config_port_0 = 0b00000110;
const uint8_t iox_config_port_1 = 0b00000111;
const uint8_t iox_drive_strength_register_00 = 0b01000000;
const uint8_t iox_drive_strenght_register_01 = 0b01000001;
const uint8_t iox_drive_strenght_register_10 = 0b01000010;
const uint8_t iox_drive_strenght_register_11 = 0b01000011;
const uint8_t iox_in_latch_register_0 = 0b01000100;
const uint8_t iox_in_ltach_register_1 =0b01000101;
const uint8_t iox_pull_up_down_en_register_0 = 0b01000110;
const uint8_t iox_pull_up_down_en_register_1 = 0b01000111;
const uint8_t iox_pull_up_down_sel_register_0 = 0b01001000;
const uint8_t iox_pull_up_down_sel_register_1 = 0b01001001;
const uint8_t iox_int_mask_register_0 = 0b01001010;
const uint8_t iox_int_mask_register_1 = 0b01001011;
const uint8_t iox_int_stat_register_0 = 0b01001100;
const uint8_t iox_int_stat_register_1 = 0b01001101;
const uint8_t iox_out_port_config_register = 0b01001111;

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
// struct pin onboard_led = {0b0, 0, 2, true};
struct pin wifi_led_g = {0b00000010, 1, 16, false};
struct pin wifi_led_or = {0b00000100, 1, 15, false};
struct pin wifi_led_r = {0b00001000, 1, 14, false};
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
float adc_offset = -122;

//pid gain
float Kp = 1;  //proportional
float Ki = 0.1;  //integral
float Kd = 0.5;  //derivative
float DT = 0.01; //pid measurement time (seconds)

// reflow cureve defines
// times define the START time of a given curve segment
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
struct curve curve_63_37 = {0, 100, 30000, 150, 120000, 183, 150000, 235, 210000, 25, 327000};


// 7 seg diplay defines
const uint8_t disp_base_add_w = 0b00000000;
const uint8_t disp_base_add_r = 0b00000001;
const uint8_t disp_1_add_w = 0b00000100;
const uint8_t disp_1_add_r = 0b00000101;
const uint8_t disp_2_add_w = 0b00000010;
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

// char bytes for 7 segment display
// dp is autimaticly attched durring the disp_write function
// segment map {0b, DP, A, B, C, D, E, F, G}
//   a
//  ___
// f|g|b
//  ___
// e|d|c
//  ___DP
const uint8_t char_0 = 0b01111110;
const uint8_t char_1 = 0b00110000;
const uint8_t char_2 = 0b01101101;
const uint8_t char_3 = 0b01111001;
const uint8_t char_4 = 0b00110011;
const uint8_t char_5 = 0b01011011;
const uint8_t char_6 = 0b01011111;
const uint8_t char_7 = 0b01110000;
const uint8_t char_8 = 0b01111111;
const uint8_t char_9 = 0b01111011;
const uint8_t char_dp = 0b10000000;
const uint8_t char_dash = 0b00000001;
const uint8_t char_a = 0b01111101;
const uint8_t char_b = 0b00011111;
const uint8_t char_c = 0b00001101;
const uint8_t char_e = 0b01001111;
const uint8_t char_f = 0b01000111;
const uint8_t char_g = 0b01011110;
const uint8_t char_h = 0b00010111;
const uint8_t char_i = 0b00000110;
const uint8_t char_j = 0b01011000;
const uint8_t char_k = 0b01010111;
const uint8_t char_l = 0b00001110;
const uint8_t char_m = 0b01010101;
const uint8_t char_n = 0b00010101;
const uint8_t char_o = 0b00011101;
const uint8_t char_p = 0b01100111;
const uint8_t char_q = 0b01110011;
const uint8_t char_r = 0b00000101;
const uint8_t char_s = 0b01101101;
const uint8_t char_t = 0b00001111;
const uint8_t char_u = 0b00011100;
const uint8_t char_v = 0b00101010;
const uint8_t char_w = 0b00101011;
const uint8_t char_x = 0b00010100;
const uint8_t char_y = 0b00111011;
const uint8_t char_z = 0b01101100;
const uint8_t char_space = 0b00000000;
const uint8_t char_qwst = 0b11110010; // already includes decimal



// reads current pin state of the io expander outputs
uint8_t read_current_io_state (int port) {
  
  //begin i2c
  Wire.beginTransmission(iox_write_add);

  // selct io port
  if (port == 0){
    Wire.write(iox_write_port_0);
    Wire.endTransmission();
  } else {
    Wire.write(iox_read_port_1);
    Wire.endTransmission();
  }

  // read current state
  Wire.requestFrom(iox_read_add, 1);
  uint8_t readval = 0b0;
  while (Wire.available()) {
    readval = Wire.read();
  }
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
    if (read_write == write) {
       // If the pin is offboard, check if we need to read or write
      if (pin_needed.port == 0) { //write to port 0
      // If we need to write, check which port the pin is on
        uint8_t current = read_current_io_state(pin_needed.port);
        if (high_low == high) {
          uint8_t highval = current | pin_needed.mask;
          Wire.beginTransmission(iox_write_add);
          Wire.write(iox_write_port_0);
          Wire.write(highval); //not sure chek this
          Wire.endTransmission();
        } else if (high_low == low) {
          uint8_t lowval = current & pin_needed.mask;
          Wire.beginTransmission(iox_write_add);
          Wire.write(iox_write_port_0);
          Wire.write(lowval);
          Wire.endTransmission();
        } else {
          //need something here probly
        }
      } else if (pin_needed.port == 1) { // write to port 1
      // If we need to write, check which port the pin is on
        uint8_t current = read_current_io_state(pin_needed.port);
        if (high_low == high) {
          uint8_t highval = current | pin_needed.mask;
          Wire.beginTransmission(iox_write_add);
          Wire.write(iox_write_port_1);
          Wire.write(highval); //not sure chek this
          Wire.endTransmission();
        } else if (high_low == low) {
          uint8_t lowval = current | pin_needed.mask;
          Wire.beginTransmission(iox_write_add);
          Wire.write(iox_write_port_1);
          Wire.write(lowval); //not sure chek this
          Wire.endTransmission();
        } else {
          //need something here probly
        }
      } else {
        //need something here probly
      }
    } else if (read_write == read) {
      if (pin_needed.port == 0) {  // write read from port 0
      // If we need to read, check which port the pin is on
        Wire.beginTransmission (iox_write_add);
        Wire.write(iox_read_port_0);
        Wire.endTransmission();

        Wire.requestFrom(iox_read_add, 1); // read from port
        uint8_t readval = 0b0;
        while (Wire.available()) {
        readval = Wire.read();
        }
        if (pin_needed.mask & readval == 0){
          return(LOW);
        } else {
          return(HIGH);
        }
        
      } else if (pin_needed.port == 1) {
        // If we need to read, check which port the pin is on
        Wire.beginTransmission (iox_write_add);
        Wire.write(iox_read_port_1);
        Wire.endTransmission();

        Wire.requestFrom(iox_read_add, 1);  //read from port
        uint8_t readval = 0b0;
        while (Wire.available()) {
        readval = Wire.read();
        }
        if (pin_needed.mask & readval == 0) {
          return(LOW);
        } else {
          return(HIGH);
        }

      }
    }
  } else { // return if input bad
    
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
    delay (0);
  }

  total_readings = total_readings / number_of_readings; // avrige the readings
  
  float voltage = total_readings * 1 / 1023.0;// convert to voltage    (the multiplication componat is the source voltage with is currently 1)
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

  float output = Kp * error + Ki * integral + Kd * derivitve;

  //output = constrain (output, -1.0, 1.0);
  output = map (output, low_range, high_range, -1, 1);

  return (output);

}

// display write function
// to_write: string to be witten to the display
// currently only supports max 16 cahricter input as of right now will add sepping at a later time
char disp_write(char* to_write) {

  Serial.println("display write called");

  // defines
  int to_write_len = strlen(to_write);
  char string[to_write_len];
  int decimal_counter = 0;
  int decimal_pos[to_write_len];
  uint8_t dig_byte[to_write_len];

  for (int j = 0; j < to_write_len; j++) { // write all 0s to the decimal and dig_dyte array
    decimal_pos[j] = 0;
    dig_byte[j] = 0b00000000;
  }
  

  for (int i = 0; i < to_write_len; i++) {  // load input sting to array
    // will load and given cahricter of the input sting in to an array
    // includes logic to bind the approprite designater for spaces decimal points dashes and qwestion marks
    if (to_write[i] == '.') {
      decimal_counter++;
      decimal_pos[(i - 1) - decimal_counter] = 1;
    } else if (to_write[i] == ' ') {
      string[i] = 'space';
    } else if (to_write[i] == '-') {
      string[i] = 'dash';
    } else if (to_write[i] == '?') {
      string[i] = 'qwst';
    } else {
      string[i] = to_write[i];
    }
  }

  for (int k = 0; k < to_write_len; k++) {  // translate form text to bytes
    char current_char = 'char_' + string[k];
    if (decimal_pos[k] == 1){  // attach decimal point if needed
      dig_byte[k] = current_char | char_dp;
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

  Serial.println("disp 2 line write called");

  int line_1_len = log10(line_1);
  std::string line_1_str = std::to_string(line_1);
  std::string line_2_str = std::to_string(line_2);
  std::string concat = line_1_str;
  for (int i = 0; i < 8 - line_1_len; i++) {
    concat += " ";
  }
  concat += line_2_str;
  char* out = new char[concat.length() + 1];
  strcpy(out, concat.c_str());
  disp_write(out);
  delete[] out;
}

// display blank
// just writes 0s to all digit registers on both display drivers resulting in no segments being on
void disp_blank() {

  Serial.println("display blanked");

  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_0);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_1);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_2);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_3);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_4);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_5);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_6);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_1_add_w);
  Wire.write(disp_digit_7);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_0);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_1);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_2);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_3);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_4);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_5);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_6);
  Wire.write(char_space);
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_digit_7);
  Wire.write(char_space);
  Wire.endTransmission();
}

// heater control
// input: takes a float input from -1 to 1 and based on that turns on or off the heaters
void heater_control (float input) {
  if (input > 0 && input < 0.5) {
    io_call (htr_1, write, high);
    io_call (htr_2, write, low);
  } else if (input > 0.5) {
    io_call (htr_1, write, high);
    io_call (htr_2, write, high);
  } else if (input < 0) {
    io_call (htr_1, write, low);
    io_call (htr_2, write, low);
  }

}

// reflow control function
// curve_needed: struct of reflow cure to follow
int reflow_control (struct curve curve_needed) {

  // turn on oven fan
  io_call(fan, write, high);

  Serial.println("refow loop enterd");

  // start timer
  unsigned long start_time;
  start_time = millis();

  // exit buttion defines
  unsigned long stop_time;
  bool exit_state = false;
  
  
  // control loop
  while ((millis() - start_time) < curve_needed.end_time) {

    // clear display
    disp_blank();
    
    if ((millis() - start_time) > curve_needed.preheat_time  && (millis() - start_time) < curve_needed.soak_time) {  // preheat stage
      heater_control(pid(curve_needed.preheat_temp));
      if ((millis() - start_time) < 3000){
        disp_write("pre-heat stage");
      } else {
        disp_write_2_line(current_temp(), curve_needed.preheat_temp);
      }
    } else if ((millis() - start_time) > curve_needed.soak_time && (millis() - start_time) < curve_needed.ramp_to_reflow_time) {  // soak stage
      heater_control(pid(curve_needed.soak_temp));
      if ((millis() - start_time) < (curve_needed.soak_time + 3000)){
        disp_write("soak stage");
      } else {
        disp_write_2_line(current_temp(), curve_needed.soak_temp);
      }
    } else if ((millis() - start_time) > curve_needed.ramp_to_reflow_time && (millis() - start_time) < curve_needed.reflow_time) {  // ramp to reflow stage
      heater_control(pid(curve_needed.ramp_to_reflow_temp));
      if ((millis() - start_time) < (curve_needed.ramp_to_reflow_time + 3000)){
        disp_write("ramp stage");
      } else {
        disp_write_2_line(current_temp(), curve_needed.ramp_to_reflow_temp);
      }
    } else if ((millis() - start_time) > curve_needed.reflow_time && (millis() - start_time) < curve_needed.cooling_time) {  // reflow stage
      heater_control(pid(curve_needed.reflow_temp));
      if ((millis() - start_time) < (curve_needed.reflow_time + 3000)){
        disp_write("reflow stage");
      } else {
        disp_write_2_line(current_temp(), curve_needed.reflow_temp);
      }
    } else if ((millis() - start_time) > curve_needed.cooling_time && (millis() - start_time) < curve_needed.end_time) {  // cooling stage
      heater_control(pid(curve_needed.cooling_temp));
      if ((millis() - start_time) < (curve_needed.cooling_time + 3000)){
        disp_write("cooling stage");
      } else {
        disp_write_2_line(current_temp(), curve_needed.cooling_temp);
      }

    }

    if (io_call(ent_btn, read, read_mode) == LOW) { // exit reflow loop early if 'ent' buttion is held for 3 seconds
      if (!exit_state) {
        stop_time = millis();
        exit_state = true;
      } else {
        if ((millis() - stop_time) >= 3000) {
          // turn off oven fan
          io_call(fan, write, low);
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
  Wire.begin(sda.pin_number, scl.pin_number, 400000);
  Serial.println("i2c started");
  
  // io epander init
  // set all io expander outputs low
  Wire.beginTransmission(iox_write_add);
  Wire.write(iox_write_port_0);  //port 0
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(iox_write_add);
  Wire.write(iox_write_port_1);  //port 1
  Wire.write(0b00000000);
  Wire.endTransmission();
  Serial.println("io expander all pins set to low");

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
  // might need to pull out of shutdown mode before self address??????????????

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
  Wire.write(0b10000000);  // 9/16 duty cycle  might want to chek if this is right
  Wire.endTransmission();
  Wire.beginTransmission(disp_2_add_w);
  Wire.write(disp_global_intensity);
  Wire.write(0b10000000);  // 9/16 duty cycle
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

  disp_write("8888888888888888");
  delay(10000);
 
}




void loop() {

  int menu_opt = 0;
  int menu_opt_count = 1;  // rember indexing starts at 0 stupid

  // menu controlls
  if (debounce(up_btn) == LOW) { // incriment menu counter when up btn is pressed
    menu_opt++;
    Serial.println("menu up");
    if (menu_opt > menu_opt_count) { // foldback menu option count 
      menu_opt = 0;
      Serial.println("menu up foldback");
    }
  } else if (debounce(dwn_btn) == LOW) {  // decriment menu counter when up dwn is pressed
    menu_opt--;
    Serial.println("menu dwn");
    if (menu_opt < 0) { // foldback menu option count
      menu_opt = menu_opt_count;
      Serial.println("menu dwn foldback");
    }
  }

  // menu main function
  if (menu_opt == 0) {
    disp_write("63 37 curve");
    Serial.println("dispwrite?");
    delay(1000);
    if (debounce(ent_btn) == LOW) {
      reflow_control(curve_63_37);
    }
  } else if (menu_opt == 1) {
    disp_write("display ip?");
    Serial.println("dispwrite?1");
    delay (1000);
    if (debounce(ent_btn) == LOW) {
      IPAddress ip = WiFi.localIP();
      String ip_str = ip.toString();
      disp_write((char*)ip_str.c_str());  // might not work lookin to it????????
      Serial.println("dispwrite?2");
      delay (3000);
    }
  } else {
    Serial.println("else");
  }
  
  Serial.println(HEX, read_current_io_state(1));

  Serial.println("main loop");
  Serial.println(current_temp());
  delay (1000);
  



}