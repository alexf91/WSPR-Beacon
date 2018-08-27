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

#ifndef WSPR_H
#define WSPR_H

#define WSPR_BUFFER_SIZE     21
#define WSPR_SYMBOL_TIME    683
#define WSPR_SYMBOL_COUNT   162


/**
 * @brief Encode a WSPR message
 * @param result Buffer of size WSPR_BUFFER_LEN in which the result is stored.
 * @param call Callsign as string in the format AABCCC
 * @param loc 4 character QTH locator as string
 * @param power Power in dBm
 */
int8_t wspr_encode(uint8_t *result, const char *call, const char *loc, int power);


/**
 * @brief Combine the encoded message with the synchronization vector and
 * return the tone that is sent over the air.
 * @param enc Encoded message
 * @param index Index of the symbol.
 * @returns Number of the tone that is sent (0-3)
 */
uint8_t wspr_get_tone(const uint8_t *enc, uint8_t index);


#endif /* WSPR_H */
