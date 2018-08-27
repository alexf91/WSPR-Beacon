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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "beacon.h"
#include "i2c_master.h"
#include "si5351.h"
#include "wspr.h"
#include "debug.h"


/* Structure containing the internal configuration of the beacon. This is kept
 * separately from the state, because we might save it to the EEPROM in the
 * future.
 */
static struct {
    char callsign[BEACON_CALL_LEN]; /* Space padded callsign */
    int32_t lat;                    /* Latitude in 8.24 fixed point format */
    int32_t lon;                    /* Longitude in 8.24 fixed point format */
    int32_t freq_corr;              /* Frequency correction in parts per 10 million */

    struct {
        uint32_t frequency;         /* Tuning frequency of the transmitter */
        uint8_t drive;              /* Drive strength of the transmitter (2-8 mA) */
    } output[BEACON_CLK_OUTPUTS];
} beacon_config;


enum BeaconMode {
    MODE_STOPPED = 0,
    MODE_TONE = 1,
    MODE_WSPR = 2,
};

static struct {
    uint8_t mode;

    uint8_t wspr_index;
    uint8_t wspr_data[WSPR_BUFFER_SIZE];
} beacon_state[BEACON_CLK_OUTPUTS];


/* Counters used by the beacon state machine for measuring time. */
static volatile uint32_t counter[BEACON_CLK_OUTPUTS] = {0};
ISR(TIMER0_COMPA_vect) {
    for (int i = 0; i < BEACON_CLK_OUTPUTS; i++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (counter[i])
                counter[i]--;
        }
    }
}


/**
 * @brief Set the value of a counter.
 * @param out Index of the output associated with the counter.
 * @param msecs Value the counter is set to.
 */
static void set_timer(uint8_t out, uint32_t msecs) {
    if (out < BEACON_CLK_OUTPUTS) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            counter[out] = msecs;
        }
    }
}


/**
 * @brief Get the value of a counter.
 * @param out Index of the output associated with the counter.
 * @returns Value of the counter.
 */
static uint32_t get_timer(uint8_t out) {
    if (out < BEACON_CLK_OUTPUTS) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            return counter[out];
        }
    }
    return -1;
}


int8_t beacon_init(void) {
    i2c_init();
    si5351_init(SI5351_CRYSTAL_LOAD_8PF, SI5351_CLK_SRC_XTAL);

    /* Check if the clock generator is connected. */
    uint8_t dummy;
    if (si5351_read(0, &dummy))
        return 1;

    for (int i = 0; i < BEACON_CLK_OUTPUTS; i++) {
        si5351_drive_strength(i, SI5351_DRIVE_2MA);
        si5351_output_enable(i, false);
    }

    /* Configure timer 0 to a rate of 1 kHz */
    TCCR0A = (1 << WGM01);                  // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00);     // Prescaler 64
    OCR0A = 250 - 1;                        // 1 ms
    TIMSK0 = (1 << OCIE0A);                 // Compare interrupt

    return 0;
}


void beacon_poll(void) {
    for (int i = 0; i < BEACON_CLK_OUTPUTS; i++) {
        if (beacon_state[i].mode == MODE_TONE) {
            if (get_timer(i) == 0) {
                beacon_state[i].mode = MODE_STOPPED;
                si5351_output_enable(i, false);
                debug("stop tone on clk%d\n", i);
            }
        } else if (beacon_state[i].mode == MODE_WSPR) {
            if (get_timer(i) == 0) {
                if (++beacon_state[i].wspr_index < WSPR_SYMBOL_COUNT) {
                    uint8_t index = beacon_state[i].wspr_index;
                    uint8_t tone = wspr_get_tone(beacon_state[i].wspr_data, index);
                    uint32_t f = beacon_config.output[i].frequency + (3 * tone) / 2;
                    si5351_set_freq(f, i);
                    set_timer(i, WSPR_SYMBOL_TIME);

                    debug("Transmitting tone %d: %d\n", index, tone);
                } else {
                    si5351_output_enable(i, false);
                    beacon_state[i].mode = MODE_STOPPED;
                }
            }
        }
    }

}


int8_t beacon_set_freq(uint8_t out, uint32_t freq) {
    if (out >= BEACON_CLK_OUTPUTS)
        return 1;

    beacon_config.output[out].frequency = freq;
    debug("frequency (clk%d) = %ld Hz\n", out, freq);
    return 0;
}


int8_t beacon_set_correction(int32_t pptm) {
    si5351_set_correction(pptm);
    debug("frequency correction = %ld pptm\n", pptm);
    return 0;
}


int8_t beacon_set_drive_strength(uint8_t out, uint8_t drive) {
    if (out >= BEACON_CLK_OUTPUTS)
        return 1;

    if (drive & 1 || drive < 2 || drive > 8)
        return 2;

    debug("drive (clk%d) = %d mA\n", out, drive);
    si5351_drive_strength(out, drive / 2 - 1);
    return 0;
}


int8_t beacon_set_callsign(uint8_t index, char c) {
    if (index >= BEACON_CALL_LEN)
        return 1;

    // TODO: check for value of c
    beacon_config.callsign[index] = c;
    debug("call[%d] = '%c'\n", index, c);

    return 0;
}


int8_t beacon_set_latitude(int32_t lat) {
    beacon_config.lat = lat;
    return 0;
}


int8_t beacon_set_longitude(int32_t lon) {
    beacon_config.lon = lon;
    return 0;
}


int8_t beacon_transmit_tone(uint8_t out, uint32_t msecs) {
    if (out >= BEACON_CLK_OUTPUTS)
        return 1;

    beacon_state[out].mode = MODE_TONE;
    si5351_set_freq(beacon_config.output[out].frequency, out);
    si5351_output_enable(out, true);
    set_timer(out, msecs);

    debug("start tone on clk%d for %ld ms\n", out, msecs);

    return 0;
}


int8_t beacon_transmit_wspr(uint8_t out) {
    if (out >= BEACON_CLK_OUTPUTS)
        return 1;

    if (wspr_encode(beacon_state[out].wspr_data, beacon_config.callsign, "JN68", 7)) {
        return 2;
    }

    beacon_state[out].mode = MODE_WSPR;
    beacon_state[out].wspr_index = 0;

    /* Start transmitting the first tone. */
    uint8_t tone = wspr_get_tone(beacon_state[out].wspr_data, 0);
    uint32_t f = beacon_config.output[out].frequency + (3 * tone) / 2;
    si5351_set_freq(f, out);
    si5351_output_enable(out, true);
    set_timer(out, WSPR_SYMBOL_TIME);

    debug("Transmitting tone 0: %d\n", tone);

    return 0;
}


int8_t beacon_transmit_aprs(uint8_t out) {
    if (out >= BEACON_CLK_OUTPUTS)
        return 1;

    return 0;
}
