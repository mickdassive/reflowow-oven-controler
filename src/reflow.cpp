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

#ifndef reflow_cpp
#define reflow_cpp

#include <Arduino.h>
#include "debug.h"
#include "io.h"
#include "reflow.h"
#include "display.h"


//pid gain
float Kp = 0.5;  //proportional
float Ki = 0.1;  //integral
float Kd = 0.5;  //derivative
float DT = 0.01; //pid measurement time (seconds)


/**
 * @brief Calculates the PID control output for the reflow oven temperature.
 *
 * This function computes the PID (Proportional-Integral-Derivative) control output
 * based on the given setpoint and the current temperature read from the system.
 * The output is constrained within a specified range and then mapped to a normalized
 * value between -1 and 1.
 *
 * @param setpoint The desired temperature setpoint for the reflow process.
 * @return float The normalized PID output in the range [-1, 1].
 */
float reflow_pid(float setpoint) {

    debug_msg(partal_reflow, "reflow_pid called", false, 0);

  float error = setpoint - io_read_current_temp(); // get the error between setpoint and current temp
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


/**
 * @brief Controls the reflow oven process according to the specified temperature curve.
 *
 * This function manages the entire reflow soldering process, including preheat, soak, ramp to reflow,
 * reflow, and cooling stages. It interacts with hardware components such as the oven fan, heater,
 * lights, and status LEDs, and provides user feedback via display and serial output.
 *
 * The function waits for the user to load the oven and confirm via the 'enter' button before starting.
 * It then progresses through each stage based on temperature and time requirements defined in the
 * provided curve structure. The process can be aborted early by holding the 'enter' button for 3 seconds.
 *
 * @param curve_needed The temperature and timing profile for the reflow process, provided as a struct curve.
 *
 * @note This function blocks until the reflow process is complete or aborted by the user.
 * @note Hardware-specific functions (e.g., io_call, io_heater_control, disp_write) are used for I/O operations.
 */
void reflow_control(struct curve curve_needed) {

    debug_msg(partal_reflow, "reflow_control called", false, 0);

    // Turn on oven fan and light, change status LED
    io_call(fan, write, high);
    io_call(ovlight, write, high);
    io_call(status_led_g, write, low);
    io_call(status_led_r, write, high);

    // State variables
    unsigned long stop_time = 0;
    bool exit_state = false;
    bool exit_trigger = false;

    unsigned long pre_heat_time = 0;
    bool pre_heat_complete = false;

    unsigned long soak_time = 0;
    bool soak_complete = false;

    unsigned long ramp_time = 0;
    bool ramp_complete = false;

    unsigned long reflow_time = 0;
    bool reflow_complete = false;

    unsigned long cooling_time = 0;
    bool cooling_complete = false;

    // Wait for preheat
    while (io_debounce(ent_btn) == HIGH) {
        debug_msg(partal_reflow, "current temp", true, io_read_current_temp());

        if (io_read_current_temp() < curve_needed.preheat_temp) {
            io_heater_control(reflow_pid(curve_needed.preheat_temp));
            disp_write("please wait");
            delay(1000);
            disp_write("preheating");
            delay(1000);
        } else {
            io_heater_control(reflow_pid(curve_needed.preheat_temp));
            disp_write("preheat complete");
            delay(1000);
            disp_write("hold enter");
            delay(1000);
            disp_write("when oven loaded");
            delay(1000);
        }
    }

    // Blink lights between stages
    for (int i = 0; i < 3; ++i) {
        io_call(ovlight, write, high);
        delay(200);
        io_call(ovlight, write, low);
        delay(200);
    }
    io_call(ovlight, write, high);

    // Start timer
    unsigned long start_time = millis();

    // Main control loop
    while (!exit_trigger) {
        // --- Preheat Stage ---
        if (!pre_heat_complete) {
            io_heater_control(reflow_pid(curve_needed.preheat_temp));
            if ((millis() - start_time) < 3000) {
                disp_write("pre-heat stage");
            } else {
                disp_write_2_line(io_read_current_temp(), curve_needed.preheat_temp);
            }
            if ((curve_needed.preheat_temp < io_read_current_temp()) && (pre_heat_time == 0)) {
                pre_heat_time = millis();
            }
            if (((millis() - pre_heat_time) > static_cast<unsigned long>(curve_needed.preheat_time)) && (pre_heat_time != 0)) {
                pre_heat_complete = true;
            }
        }
        // --- Soak Stage ---
        else if (!soak_complete && pre_heat_complete) {
            io_heater_control(reflow_pid(curve_needed.soak_temp));
            unsigned long soak_stage_time = millis() - (pre_heat_time + curve_needed.preheat_time);
            if (soak_stage_time < 3000) {
                disp_write("soak stage");
                if (soak_stage_time < 1000) {
                    io_call(ovlight, write, low);
                } else if (soak_stage_time > 2000) {
                    io_call(ovlight, write, high);
                }
            } else {
                disp_write_2_line(io_read_current_temp(), curve_needed.soak_temp);
            }
            if ((curve_needed.soak_temp > io_read_current_temp()) && (soak_time == 0)) {
                soak_time = millis();
            }
            if (((millis() - soak_time) > static_cast<unsigned long>(curve_needed.soak_time)) && (soak_time != 0)) {
                soak_complete = true;
            }
        }
        // --- Ramp to Reflow Stage ---
        else if (pre_heat_complete && soak_complete && !ramp_complete) {
            io_heater_control(reflow_pid(curve_needed.ramp_to_reflow_temp));
            unsigned long ramp_stage_time = millis() - (soak_time + curve_needed.soak_time);
            if (ramp_stage_time < 3000) {
                disp_write("ramp stage");
                if (ramp_stage_time < 1000) {
                    io_call(ovlight, write, low);
                } else if (ramp_stage_time > 2000) {
                    io_call(ovlight, write, high);
                }
            } else {
                disp_write_2_line(io_read_current_temp(), curve_needed.ramp_to_reflow_temp);
            }
            if ((curve_needed.ramp_to_reflow_temp > io_read_current_temp()) && (ramp_time == 0)) {
                ramp_time = millis();
            }
            if (((millis() - ramp_time) > static_cast<unsigned long>(curve_needed.ramp_to_reflow_time)) && (ramp_time != 0)) {
                ramp_complete = true;
            }
        }
        // --- Reflow Stage ---
        else if (pre_heat_complete && soak_complete && ramp_complete && !reflow_complete) {
            io_heater_control(reflow_pid(curve_needed.reflow_temp));

            unsigned long reflow_stage_time = millis() - (ramp_time + curve_needed.ramp_to_reflow_time);
            if (reflow_stage_time < 3000) {
                disp_write("reflow stage");
                if (reflow_stage_time < 1000) {
                    io_call(ovlight, write, low);
                } else if (reflow_stage_time > 2000) {
                    io_call(ovlight, write, high);
                }
            } else {
                disp_write_2_line(io_read_current_temp(), curve_needed.reflow_temp);
            }
            if ((io_read_current_temp() > curve_needed.reflow_temp) && (reflow_time == 0)) {
                reflow_time = millis();
            }
            if (((millis() - reflow_time) > static_cast<unsigned long>(curve_needed.reflow_time)) && (reflow_time != 0)) {
                reflow_complete = true;
            }
        }
        // --- Cooling Stage ---
        else if (pre_heat_complete && soak_complete && ramp_complete && reflow_complete && !cooling_complete) {
            io_heater_control(reflow_pid(curve_needed.cooling_temp));

            if (cooling_time == 0) {
                cooling_time = millis();
            }
            unsigned long cooling_stage_time = millis() - (reflow_time + curve_needed.reflow_time);
            if (cooling_stage_time < 3000) {
                disp_write("cooling stage");
                if (cooling_stage_time < 1000) {
                    io_call(ovlight, write, low);
                } else if (cooling_stage_time > 2000) {
                    io_call(ovlight, write, high);
                }
            } else {
                disp_write_2_line(io_read_current_temp(), curve_needed.cooling_temp);
            }
            if (io_read_current_temp() <= curve_needed.cooling_temp) {
                cooling_complete = true;
            }
            // cooling_complete = true;
        }
        // --- End of Reflow Process ---
        else if (pre_heat_complete && soak_complete && ramp_complete && reflow_complete && cooling_complete) {
            exit_trigger = true;
        }

        // Early exit if 'ent' button is held for 3 seconds
        if (io_debounce(ent_btn) == LOW) {
            if (!exit_state) {
                stop_time = millis();
                exit_state = true;
            } else if ((millis() - stop_time) >= 3000) {
                // Return to default state
                for (int i = 0; i < 3; ++i) {
                    io_call(status_led_g, write, low);
                    delay(100);
                    io_call(status_led_g, write, high);
                    delay(100);
                }
                io_call(fan, write, low);
                io_call(ovlight, write, low);
                io_call(status_led_g, write, high);
                io_call(status_led_r, write, low);
                io_call(htr_1, write, low);
                return;
            }
        } else {
            exit_state = false;
        }

        delay(100); // Loop runs every 1/10th second
    }

    // Turn off oven fan and reset outputs
    io_call(fan, write, low);
    io_call(ovlight, write, low);
    io_call(status_led_g, write, high);
    io_call(status_led_r, write, low);
    io_call(htr_1, write, low);
    io_call(htr_2, write, low);

    return;
}

// oven preheat/hold
void reflow_temp_hold () {

  int setpoint = 0;

  disp_write("enter set-point");

  delay(3000);

  while (io_debounce(ent_btn) == HIGH) {
    if (io_debounce(up_btn) == LOW){
      setpoint++;
    } else if (io_debounce(dwn_btn) == LOW) {
      setpoint--;
    }
    
    debug_msg(partal_reflow, "setpoint", true, setpoint);

    disp_write_2_line(setpoint, setpoint);
  }

  io_call(status_led_g, write, low);
  io_call(status_led_r, write, high);
  io_call(ovlight, write, high);

  io_call(fan, write, high);

  disp_write("enter to exit");
  delay(2000);
  disp_write("begin pre-heat");
  delay(3000);

  while (io_debounce(ent_btn) == HIGH) {
    if (io_read_current_temp() > (setpoint + 10) && io_read_current_temp() < (setpoint - 10)) {
      disp_write("done heating");
      io_call(ovlight, write, high);
      delay (1000);
      io_call(ovlight, write, low);
    } else {
      io_heater_control(reflow_pid(setpoint));
      disp_write_2_line(io_read_current_temp(), setpoint);
    }

  }

  delay(1000);
  io_call(status_led_r, write, low);
  io_call(ovlight, write, low);
  io_call(fan, write, low);
return;
}





#endif // reflow_cpp