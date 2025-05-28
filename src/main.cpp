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

#ifndef main_cpp
#define main_cpp


#include <Arduino.h>
#include <Wire.h>
#include "io.h"
#include "display.h"
#include "debug.h"
#include "reflow.h"

const char* firmware_v = "firmware v 0.3";

enum MenuOptions {
  CURVE_63_37,
  DISPLAY_IP,
  FIRMWARE_VERSION,
  SET_HOLD
};

static MenuOptions menuOption = CURVE_63_37;
static const char* menuLabels[] = {
  "63 37 curve",
  "display ip ?",
  firmware_v,
  "set-point hold"
};

static const int menuCount = sizeof(menuLabels) / sizeof(menuLabels[0]);

// Track if a menu option is selected
static bool menuSelected = false;

void setup() {

  // start serial
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("serial started");

  // start i2c
  Wire.begin(sda.pin_number, scl.pin_number);
  Wire.setClock(400000); //
  debug_msg(partal_io, "i2c started", false, 0);
  
  // io epander init
  io_gpio_init();
  debug_msg(partal_io, "io expander init complete", false, 0);


  


}


void loop() {

  io_call(status_led_g, write, high);

  // display current menu option or indicate selection
  if (!menuSelected) {
    disp_write(menuLabels[menuOption]);
  } else {
    disp_write("Selected...");
  }

  // handle button presses for menu navigation and selection
  if (!menuSelected) {
    if (io_debounce(up_btn) == LOW) {
      menuOption = static_cast<MenuOptions>((menuOption + 1) % menuCount);
      delay(200); // debounce delay
    } else if (io_debounce(dwn_btn) == LOW) {
      menuOption = static_cast<MenuOptions>((menuOption - 1 + menuCount) % menuCount);
      delay(200); // debounce delay
    } else if (io_debounce(ent_btn) == LOW) {
      menuSelected = true;
      delay(200); // debounce delay
    }
  } else {
    // Wait for enter button to be pressed again to execute
    if (io_debounce(ent_btn) == LOW) {
      switch (menuOption) {
        case CURVE_63_37:
          reflow_control(curve_63_37);
          break;
        case DISPLAY_IP:
          disp_write("192.168.202.202");
          delay(3000);
          break;
        case FIRMWARE_VERSION:
          disp_write(firmware_v);
          delay(1500);
          break;
        case SET_HOLD:
          reflow_temp_hold();
          break;
        default:
          disp_write("error");
          break;
      }
      menuSelected = false; // Reset selection after execution
      delay(200); // debounce delay
    }
    // Optionally, allow cancel with another button (e.g., down button)
    if (io_debounce(dwn_btn) == LOW) {
      menuSelected = false;
      delay(200); // debounce delay
    }
  }
}

#endif // main_cpp


