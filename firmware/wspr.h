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

/**
 * @brief Encode a WSPR message
 * @param call Callsign as string in the format AABCCC
 * @param loc 4 character QTH locator as string
 * @param power Power in dBm
 */
int wspr_encode(uint8_t *result, const char *call, const char *loc, int power);

#endif /* WSPR_H */
