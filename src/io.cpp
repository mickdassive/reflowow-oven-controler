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

#ifndef io_cpp
#define io_cpp

#include <Arduino.h>
#include <Wire.h>
#include "io.h"
#include "debug.h"

//variables
//iox interrupt register values
uint8_t iox_port_0_interrupt = 0xFF;
uint8_t iox_port_1_interrupt = 0xFF;

//heater state
unsigned long T_of_last_htr_state = 0;
int last_htr_state = 0;
int htr_historesis_time = 1000; // time in ms



/**
 * @brief Reads the current I/O state from the specified port via I2C.
 *
 * This function communicates with an I2C I/O expander to read the current state
 * of either port 0 or port 1. It selects the appropriate port, initiates the
 * I2C transmission, and returns the value read from the port.
 *
 * @param port The port number to read from (0 for port 0, any other value for port 1).
 * @return uint8_t The current state of the selected I/O port.
 */
uint8_t read_current_io_state (int port) {

    debug_msg(partal_io, "read_current_io_state called", false, 0);
  
  //begin i2c
  Wire.beginTransmission(iox_add);

  // selct io port
  if (port == 0){
    Wire.write(iox_output_port_0);
    Wire.endTransmission();
  } else {
    Wire.write(iox_output_port_1);
    Wire.endTransmission();
  }

  // read current state
  Wire.requestFrom(iox_add, (uint8_t)1);

  return Wire.read();
}

/**
 * @brief Performs read or write operations on a specified pin, handling both onboard and IO expander pins.
 *
 * This function abstracts the process of reading from or writing to a pin, whether it is directly connected
 * to the microcontroller (onboard) or accessed via an IO expander (offboard). For onboard pins, it uses
 * standard digitalRead/digitalWrite functions. For offboard pins, it communicates with the IO expander
 * over I2C to perform the requested operation.
 *
 * @param pin_needed   Reference to a struct describing the pin to access, including its number, port, mask, and whether it is onboard.
 * @param rw           Enum value specifying the operation: read or write.
 * @param hl           Enum value specifying the logic level: high or low (used only for write operations).
 * @return int         Returns the read value (HIGH/LOW) for read operations, 0 for successful write operations,
 *                     or -1 if an invalid operation is requested.
 */
int io_call(struct pin pin_needed, enum read_write rw, enum high_low hl) {

    debug_msg(partal_io, "io_call called", false, 0);

    // Onboard pin handling
    if (pin_needed.onboard) {
        if (rw == write) {
            if (hl == low) {
                digitalWrite(pin_needed.pin_number, LOW);
            } else if (hl == high) {
                digitalWrite(pin_needed.pin_number, HIGH);
            } else if (hl == read_mode) {
                // Invalid high_low value for write operation
                return -1;
            }
            digitalWrite(pin_needed.pin_number, HIGH);
            return 0;
        } else if (rw == read) {
            return digitalRead(pin_needed.pin_number);
        } else {
            // Invalid read_write value
            return -1;
        }
    }

    // Offboard (IO expander) pin handling
    if (rw == write) {
        uint8_t current = read_current_io_state(pin_needed.port);
        uint8_t output = (hl == high) ? (current | pin_needed.mask) : (current & ~pin_needed.mask);
        uint8_t port_reg = (pin_needed.port == 0) ? iox_output_port_0 : iox_output_port_1;

        Wire.beginTransmission(iox_add);
        Wire.write(port_reg);
        Wire.write(output);
        Wire.endTransmission();
        return 0;
    } else if (rw == read) {
        uint8_t port_reg = (pin_needed.port == 0) ? iox_input_port_0 : iox_input_port_1;

        Wire.beginTransmission(iox_add);
        Wire.write(port_reg);
        Wire.endTransmission();

        Wire.requestFrom(iox_add, (uint8_t)1);
        uint8_t readval = Wire.read();

        return (readval & pin_needed.mask) ? HIGH : LOW;
    }

    // Invalid input
    return -1;
}

/**
 * @brief Initializes the GPIO pins for input/output and sets up interrupt masks for IO expanders.
 * 
 * This function initializes the GPIO pins based on the pin mode specified for each pin. It sets the pin mode for each pin,
 * configures interrupt masks for IO expanders, and sets all outputs of IO expanders to low.
 * 
 */
void io_gpio_init() {

  debug_msg(partal_io, "io_gpio_init called, setting up pins", false, 0);

  //init vars
  static uint8_t iox_0_port_0_pinmode = 0x00;
  static uint8_t iox_0_port_1_pinmode = 0x00;

  //begin iterating through pins
  for (size_t i = 1; i < sizeof(pin_names[0]); i++) {

    switch (pin_names[i]->pin_mode) {
    default:
      // Error handling for unexpected pin_mode value
      debug_msg(partal_io, "Unexpected pin_mode value", false, 0);
      break;
    case in:
      if (pin_names[i]->onboard) {
        pinMode(pin_names[i]->pin_number, INPUT);

      }
      //no need to do anything to the iox, all pins default to input
      break;
    
    case out:
      if (pin_names[i]->onboard) {
        pinMode(pin_names[i]->pin_number, OUTPUT);

      } else {
        if (pin_names[i]->port == 0) {
            iox_0_port_0_pinmode |= pin_names[i]->mask;

        } else {
            iox_0_port_1_pinmode |= pin_names[i]->mask;

        }        
      }
      
      break;

    case bi_dir:
      //do nothing, only used for i2c coms which are defined elsewhere 
      break;
    
    case intr: //setup interrupts for checking when io state is changed
      if(pin_names[i]->onboard) {
        pinMode(pin_names[i]->pin_number, INPUT);
        //attachInterrupt(digitalPinToInterrupt(pin_names[i]->pin_number), io_pin_interrupt_flagger, RISING);

      } else {
        
        if (pin_names[i]->port == 0) {
            iox_port_0_interrupt |= ~pin_names[i]->mask;

        } else {
            iox_port_1_interrupt |= pin_names[i]->mask;

        }
      }  
      break;
    case empty_pin:
      //do nothing
      break;
    }  
  }

  //set all outputs of ioxs low
  //write to output at iox 0 port 0
  Wire.beginTransmission(iox_add);
  Wire.write(iox_output_port_0);
  Wire.write(0x00);
  Wire.endTransmission();

  //write to output at iox 0 port 1
  Wire.beginTransmission(iox_add);
  Wire.write(iox_output_port_1);
  Wire.write(0x00);
  Wire.endTransmission();

  //begin writing pinmodes to ioxs 
  //write pinmode to iox 0 port 0 
  Wire.beginTransmission(iox_add);
  Wire.write(iox_config_port_0);
  Wire.write(iox_0_port_0_pinmode);
  Wire.endTransmission();

  //write pinmode to iox 0 port 1 
  Wire.beginTransmission(iox_add);
  Wire.write(iox_config_port_1);
  Wire.write(iox_0_port_1_pinmode);
  Wire.endTransmission();


  //write to ioxs interrupt mask registers
  //write mask to iox 0 port 0
  Wire.beginTransmission(iox_add);
  Wire.write(iox_int_mask_register_0);
  Wire.write(iox_port_0_interrupt);
  Wire.endTransmission();

  //write mask to iox 0 port 1
  Wire.beginTransmission(iox_add);
  Wire.write(iox_int_mask_register_1);
  Wire.write(iox_port_1_interrupt);
  Wire.endTransmission();

  debug_msg(partal_io, "pins have been initialized", false, 0);

  return;
}

/**
 * @brief Controls the heater state based on the input value.
 *
 * This function manages the state of the heater based on the input value.
 * It uses hysteresis to prevent rapid switching between states. The function
 * checks if enough time has passed since the last state change before updating
 * the heater state.
 *
 * @param input (0 - 1) The input value used to determine the heater state.
 */
void io_heater_control (float input) {

    debug_msg(partal_io, "io_heater_control called", false, 0);

  if ((millis() - (T_of_last_htr_state)) < (unsigned long)htr_historesis_time) {
    
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
  
  return;

}

/**
 * @brief Sets the heater hysteresis time.
 *
 * This function updates the global heater hysteresis time variable with the specified value.
 * Hysteresis time is typically used to prevent rapid switching of the heater by introducing
 * a delay or buffer period before the heater can be turned on or off again.
 *
 * @param time The new hysteresis time (in milliseconds or appropriate time unit).
 */
void io_set_heater_historesis (int time) {

    debug_msg(partal_io, "io_set_heater_historesis called", true, time);

    htr_historesis_time = time;

    return;
}

/**
 * @brief Reads the current temperature from the analog temperature sensor.
 *
 * This function performs multiple analog readings from the temperature sensor pin,
 * averages the results to reduce noise, and converts the averaged ADC value to a
 * voltage. The voltage is then converted to a temperature in degrees Celsius using
 * the sensor's characteristics and an offset.
 *
 * @return The measured temperature in degrees Celsius.
 */
float io_read_current_temp() {

    debug_msg(partal_io, "io_read_current_temp called", false, 0);

  const int number_of_readings = 10; // number or readings to avrige for mesurement
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

/**
 * @brief Debounces a button press by checking its state after a short delay.
 *
 * This function reads the state of a button pin, waits for a short period to allow
 * for any bouncing to settle, and then checks the state again. If the state is still
 * HIGH, it returns HIGH; otherwise, it returns LOW.
 *
 * @param butt_pin The pin structure representing the button to debounce.
 * @return int Returns HIGH if the button is pressed (stable), LOW otherwise.
 */
int io_debounce(struct pin butt_pin) {

    debug_msg(partal_io, "debounce called", false, 0);

    bool initialState = io_call(butt_pin, read, read_mode);
    delay(50);  // debounce sampel time
    if (initialState == HIGH && io_call(butt_pin, read, read_mode)) {
        return (HIGH);
    } else {
        return (LOW);
    }
}



#endif //io_cpp