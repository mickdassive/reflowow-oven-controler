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

#ifndef display_cpp
#define display_cpp

#include <Arduino.h>
#include <Wire.h>
#include "display.h"
#include "debug.h"

//brigtness value
uint8_t disp_intense = 0xff; // set this number in hex to set 7 segment display brightness



/**
 * @brief Initializes the display modules via I2C.
 *
 * This function performs the following steps to initialize two display drivers:
 * 1. Sends shutdown and reset commands to the base display address.
 * 2. Sets the self-addressing mode for the display.
 * 3. Pulls both display drivers out of shutdown mode.
 * 4. Sets the decode mode to "raw" for both displays.
 * 5. Configures the scan limit for both displays.
 * 6. Sets the global intensity for both displays.
 * 7. Performs a self-test on both displays and then ends the test.
 *
 * Debug and status messages are printed to the serial output at each step.
 *
 * Assumes that the following variables and constants are defined elsewhere:
 * - disp_base_add_w, disp_1_add_w, disp_2_add_w: I2C addresses for the displays.
 * - disp_shutdown, disp_up_rst, disp_self_add, disp_up, disp_decode_mode,
 *   disp_scan_limit, disp_global_intensity, disp_intense, disp_test: Command codes.
 * - debug_msg(): Debug message function.
 *
 * @note This function requires the Wire library for I2C communication.
 */
void disp_init() {
    debug_msg(partal_disp, "display init called", false, 0);

    // 1. Shutdown and reset base display
    Wire.beginTransmission(disp_base_add_w);
    Wire.write(disp_shutdown);
    Wire.write(disp_up_rst);
    Wire.endTransmission();

    // 2. Set self-addressing mode
    Wire.beginTransmission(disp_base_add_w);
    Wire.write(disp_self_add);
    Wire.write(0b00000001);
    Wire.endTransmission();

    // 3. Pull both displays out of shutdown mode
    Wire.beginTransmission(disp_1_add_w);
    Wire.write(disp_shutdown);
    Wire.write(disp_up);
    Wire.endTransmission();

    Wire.beginTransmission(disp_2_add_w);
    Wire.write(disp_shutdown);
    Wire.write(disp_up);
    Wire.endTransmission();

    // 4. Set decode mode to "raw" for both displays
    Wire.beginTransmission(disp_1_add_w);
    Wire.write(disp_decode_mode);
    Wire.write(0b00000000);
    Wire.endTransmission();

    Wire.beginTransmission(disp_2_add_w);
    Wire.write(disp_decode_mode);
    Wire.write(0b00000000);
    Wire.endTransmission();

    // 5. Configure scan limit for both displays
    Wire.beginTransmission(disp_1_add_w);
    Wire.write(disp_scan_limit);
    Wire.write(0b00000111);
    Wire.endTransmission();

    Wire.beginTransmission(disp_2_add_w);
    Wire.write(disp_scan_limit);
    Wire.write(0b00000111);
    Wire.endTransmission();

    // 6. Set global intensity for both displays
    Wire.beginTransmission(disp_1_add_w);
    Wire.write(disp_global_intensity);
    Wire.write(disp_intense);
    Wire.endTransmission();

    Wire.beginTransmission(disp_2_add_w);
    Wire.write(disp_global_intensity);
    Wire.write(disp_intense);
    Wire.endTransmission();

    // 7. Perform self-test on both displays
    Wire.beginTransmission(disp_1_add_w);
    Wire.write(disp_test);
    Wire.write(0b10000000);
    Wire.endTransmission();

    Wire.beginTransmission(disp_2_add_w);
    Wire.write(disp_test);
    Wire.write(0b10000000);
    Wire.endTransmission();

    delay(500);

    // 8. End self-test
    Wire.beginTransmission(disp_1_add_w);
    Wire.write(disp_test);
    Wire.write(0b00000000);
    Wire.endTransmission();

    Wire.beginTransmission(disp_2_add_w);
    Wire.write(disp_test);
    Wire.write(0b00000000);
    Wire.endTransmission();

    debug_msg(partal_disp, "display init complete", false, 0);

    return;
}


/**
 * @brief Blanks (clears) the display by sending zero data to all segments.
 *
 * This function sends a command to both display controllers (disp_1_add_w and disp_2_add_w)
 * over the I2C bus, writing zeros to all 8 segments of each display. It also logs a debug
 * message indicating that the display is being blanked.
 *
 * Typical usage is to clear the display before updating it with new data or to turn off
 * all segments for power saving or visual effect.
 */
void disp_blank() {

    debug_msg(partal_disp, "display blanking", false, 0);

    Wire.beginTransmission(disp_1_add_w);
    for (int i = 0; i < 8; i++) {
        Wire.write(i + 1);
        Wire.write(0b00000000);
    }
    Wire.endTransmission();

    Wire.beginTransmission(disp_2_add_w);
    for (int i = 0; i < 8; i++) {
        Wire.write(i + 1);
        Wire.write(0b00000000);
    }
    Wire.endTransmission();

    return;

}


/**
 * @brief Writes a string to a multi-segment 7-segment display, handling decimal points.
 *
 * This function parses the input string, processes decimal points, translates each character
 * to its corresponding 7-segment display byte (including decimal point if present), and writes
 * the resulting bytes to the appropriate display segments over I2C.
 *
 * The display is assumed to be split across two I2C addresses, with up to 16 digits supported.
 * Decimal points are handled by setting the highest bit of the corresponding digit's byte.
 *
 * @param to_write Pointer to a null-terminated string to display. Supports digits, letters, dashes, and decimal points.
 *
 * @note Characters not found in the disp_7seg_alfbet lookup table will be displayed as blank (0).
 * @note Only the first 16 displayable characters (excluding decimal points) will be shown.
 */
void disp_write(const char* to_write) {

    debug_msg(partal_disp, "display write called", false, 0);

    disp_blank();

    int to_write_len = strlen(to_write);
    char string[to_write_len];
    int decimal_pos[to_write_len] = {0};
    uint8_t dig_byte[to_write_len] = {0};
    int decimal_shift = 0;

    // Parse input string, handle decimal points
    for (int i = 0; i < to_write_len; i++) {
        if (to_write[i] == '.') {
            if (i - (1 + decimal_shift) >= 0)
                decimal_pos[i - (1 + decimal_shift)] = 1;
            decimal_shift++;
        } else {
            string[i - decimal_shift] = to_write[i];
        }
    }

    // Translate characters to 7-segment bytes
    for (int k = 0; k < to_write_len - decimal_shift; k++) {
        uint8_t current_char = 0;
        for (size_t l = 0; l < sizeof(disp_7seg_alfbet) / sizeof(disp_7seg_alfbet[0]); l++) {
            if (string[k] == disp_7seg_alfbet[l]->simbol) {
                current_char = disp_7seg_alfbet[l]->value;
                break;
            }
        }
        // Attach decimal point if needed
        dig_byte[k] = decimal_pos[k] ? (current_char | 0b10000000) : current_char;
    }

    // Write bytes to display
    int display_len = to_write_len - decimal_shift;
    for (int l = 0; l < display_len && l < 16; l++) {
        if (l < 4) {
            Wire.beginTransmission(disp_1_add_w);
            Wire.write(l + 1);
            Wire.write(dig_byte[l]);
            Wire.endTransmission();
        } else if (l < 8) {
            Wire.beginTransmission(disp_2_add_w);
            Wire.write(l - 3);
            Wire.write(dig_byte[l]);
            Wire.endTransmission();
        } else if (l < 12) {
            Wire.beginTransmission(disp_1_add_w);
            Wire.write(l - 7);
            Wire.write(dig_byte[l]);
            Wire.endTransmission();
        } else if (l < 16) {
            Wire.beginTransmission(disp_2_add_w);
            Wire.write(l - 11);
            Wire.write(dig_byte[l]);
            Wire.endTransmission();
        } else {
            debug_msg(partal_disp, "display digit write overflow", false, 0);
        }
    }

    return;

}


/**
 * @brief Writes two floating-point values to a display, each formatted in its own 8-character field.
 *
 * This function formats two float values into a 16-character buffer, with the first value left-aligned
 * in the first 8 characters (2 decimal places), and the second value right-aligned in the last 8 characters (2 decimal places).
 * The formatted string is then sent to the display using disp_write().
 *
 * @param line_1 The first floating-point value to display (left-aligned, 8 characters wide).
 * @param line_2 The second floating-point value to display (right-aligned, 8 characters wide).
 */
void disp_write_2_line(float line_1, float line_2) {

    debug_msg(partal_disp, "display write 2 line called", false, 0);
    debug_msg(partal_disp, "line 1", true, line_1);
    debug_msg(partal_disp, "line 2", true, line_2);

    char buffer[17]; // 16 chars + null terminator
    snprintf(buffer, sizeof(buffer), "%-8.2f%8.2f", line_1, line_2);
    disp_write(buffer);

    return;
}

/**
 * @brief Sets the brightness of the display modules.
 *
 * This function updates the global display intensity value and sends the new
 * brightness setting to two display modules over the I2C bus. It uses the
 * specified brightness value to adjust the intensity of both displays.
 *
 * @param brightness The desired brightness level (0-255).
 */
void disp_set_brightness(uint8_t brightness) {
    debug_msg(partal_disp, "display set brightness called", false, 0);
    disp_intense = brightness;

    Wire.beginTransmission(disp_1_add_w);
    Wire.write(disp_global_intensity);
    Wire.write(disp_intense);
    Wire.endTransmission();

    Wire.beginTransmission(disp_2_add_w);
    Wire.write(disp_global_intensity);
    Wire.write(disp_intense);
    Wire.endTransmission();

    return;
}




#endif // display_cpp