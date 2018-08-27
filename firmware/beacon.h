/*
 * Copyright 2018 Alexander Fasching
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BEACON_H
#define BEACON_H

#define BEACON_CLK_OUTPUTS 3
#define BEACON_CALL_LEN 6

/**
 * @brief Initialize the internal data structures, the clock generator and
 * configure timer0.
 * @returns 0 on success
 */
int8_t beacon_init(void);

/**
 * @brief Update the state machine of the beacon. This should be called as
 * often as possible in the main loop.
 */
void beacon_poll(void);

/**
 * @brief Set the tuning frequency of a beacon output.
 * @param out Index of the output.
 * @param freq Frequency to tune to.
 * @returns 0 on success.
 */
int8_t beacon_set_freq(uint8_t out, uint32_t freq);

/**
 * @brief Set the frequency correction of the transmitter.
 * @param pptm Correction factor in parts per 10 million.
 * @returns 0 on success.
 */
int8_t beacon_set_correction(int32_t pptm);

/**
 * @brief Set the drive strength of a given output.
 * @param out Index of the output.
 * @param drive Drive strength in mA.
 * @returns 0 on success.
 */
int8_t beacon_set_drive_strength(uint8_t out, uint8_t drive);

/**
 * @brief Set a character of the callsign.
 * @param index Index of the character to set.
 * @param c Character.
 * @returns 0 on success.
 */
int8_t beacon_set_callsign(uint8_t index, char c);

/**
 * @brief Set the latitude of the position.
 * @param lat Latitude in signed 15.16 fixed point format.
 * @returns 0 on success.
 */
int8_t beacon_set_latitude(int32_t lat);

/**
 * @brief Set the longitude of the position.
 * @param lon Longitude in signed 15.16 fixed point format.
 * @returns 0 on success.
 */
int8_t beacon_set_longitude(int32_t lon);

/**
 * @brief Transmit a continuous wave for a given time. Transmission starts
 * immediately after the function call.
 * @param out Index of the output.
 * @param msecs Number of milliseconds the tone is transmitted.
 * @returns 0 on success.
 */
int8_t beacon_transmit_tone(uint8_t out, uint32_t msecs);

/**
 * @brief Transmit the current callsign, position and output power as WSPR
 * message. Transmission starts immediately after the function call.
 * @param out Index of the output.
 * @returns 0 on success.
 */
int8_t beacon_transmit_wspr(uint8_t out);

/**
 * @brief Transmit the current callsign and position as APRS message.
 * Transmission starts immediately after the function call.
 * @param out Index of the output.
 * @returns 0 on success.
 */
int8_t beacon_transmit_aprs(uint8_t out);


#endif /* BEACON_H */
