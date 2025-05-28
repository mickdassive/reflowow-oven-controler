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

#ifndef reflow_h
#define reflow_h

#include <Arduino.h>


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
const struct curve curve_63_37 = {0, 100, 90000, 150, 30000, 183, 30000, 260, 30000, 25, 327000};

//function defines
void reflow_temp_hold();
void reflow_control(struct curve curve_needed);


#endif // reflow_h