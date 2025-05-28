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

#ifndef debug_h
#define debug_h

#include <Arduino.h>

//var defines
enum debug_levels {
    full,
    partal_reflow,
    partal_io,
    partal_adc,
    partal_disp,
    no_debug
    };
;

//function define
void debug_msg(enum debug_levels debug_level, const char* debug_message, bool include_num, int num_include);
void debug_level_set(enum debug_levels new_debug_level);


#endif // debug_h