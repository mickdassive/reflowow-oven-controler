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

#ifndef debug_cpp
#define debug_cpp

#include <Arduino.h>
#include "debug.h"

//var defines
enum debug_levels debug_level = no_debug;

//functions

/**
 * @brief Prints a debug message with optional attached value.
 * 
 * This function prints a debug message along with the current millis() value. 
 * The debug message can be customized based on the debug level provided.
 * 
 * @param debug_level The debug level for the message.
 * @param debug_message The debug message to be printed.
 * @param include_num Flag indicating whether to include an attached value.
 * @param num_include The attached value to be printed (if include_num is true).
 */
void debug_msg(enum debug_levels debug_level, const char* debug_message, bool include_num, int num_include) {
    if (debug_level != no_debug) {
        switch (debug_level) {
            case full:
                Serial.print("debug msg at millis: ");
                Serial.print(millis());
                Serial.print("| ");
                if (include_num) {
                    Serial.print(debug_message);
                    Serial.print("  | attached value: ");
                    Serial.println(num_include);
                } else {
                    Serial.println(debug_message);
                }
                break;
            case partal_reflow:
                Serial.print("debug msg at millis: ");
                Serial.print(millis());
                Serial.print("| ");
                if (include_num) {
                    Serial.print(debug_message);
                    Serial.print("  | attached value: ");
                    Serial.println(num_include);
                } else {
                    Serial.println(debug_message);
                }
                break;
            case partal_io:
                Serial.print("debug msg at millis: ");
                Serial.print(millis());
                Serial.print("| ");
                if (include_num) {
                    Serial.print(debug_message);
                    Serial.print("  | attached value: ");
                    Serial.println(num_include);
                } else {
                    Serial.println(debug_message);
                }
                break;
            case partal_adc:
                Serial.print("debug msg at millis: ");
                Serial.print(millis());
                Serial.print("| ");
                if (include_num) {
                    Serial.print(debug_message);
                    Serial.print("  | attached value: ");
                    Serial.println(num_include);
                } else {
                    Serial.println(debug_message);
                }
                break;
            case partal_disp:
                Serial.print("debug msg at millis: ");
                Serial.print(millis());
                Serial.print("| ");
                if (include_num) {
                    Serial.print(debug_message);
                    Serial.print("  | attached value: ");
                    Serial.println(num_include);
                } else {
                    Serial.println(debug_message);
                }
                break;
            case no_debug:
                break;
        }
    }
}

void debug_level_set(enum debug_levels new_debug_level) {
    debug_level = new_debug_level;
}



#endif // debug_cpp