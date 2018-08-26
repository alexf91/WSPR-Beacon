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

#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "uart.h"
#include "usbdrv.h"
#include "i2c_master.h"
#include "si5351.h"
#include "wspr.h"


#define CLK_OUTPUTS 3
#define CALL_LENGTH 6


/* USB requests */
enum Request {
    REQ_ENABLE_LED = 0,     /* Enable the LED */
    REQ_DISABLE_LED,        /* Disable the LED */
    REQ_SET_FREQ,           /* Set the frequency of an output */
    REQ_SET_DRIVE,          /* Set drive strenght for an output */
    REQ_SET_CALL,           /* Set the callsign */
    REQ_SET_FREQ_CORR,      /* Set frequency correction value */
    REQ_START_TONE,         /* Start continuous tone */
    REQ_STOP_TONE,          /* Stop continuous tone */
};

/* Status codes */
enum Status {
    STATUS_OK = 0,
    STATUS_GENERIC_ERROR,
    STATUS_INVALID_REQUEST,
    STATUS_INVALID_INDEX,
    STATUS_INVALID_VALUE,
};

char strbuf[64];

static inline void led_init(void) { DDRB |= (1<<PB5); }
static inline void led_on(void) { PORTB |= (1<<PB5); }
static inline void led_off(void) { PORTB &= ~(1<<PB5); }


/* Structure containing the internal state of the beacon. */
struct BeaconConfig {
    char callsign[CALL_LENGTH];     /* Space padded callsign */
    int32_t lat;                    /* Latitude in 8.24 fixed point format */
    int32_t lon;                    /* Longitude in 8.24 fixed point format */
    int32_t freq_corr;              /* Frequency correction in parts per 10 million */

    struct {
        uint32_t frequency;         /* Tuning frequency of the transmitter */
        uint8_t drive;              /* Drive strength of the transmitter (2-8 mA) */
    } output[CLK_OUTPUTS];
};
struct BeaconConfig beacon_config;  /* Keep a version of the config in RAM. */


/* Structure used as response to requests. */
struct {
    uint16_t value;
    uint16_t status;
} response;


usbMsgLen_t usbFunctionSetup(uchar data[8]) {
    usbRequest_t *rq = (void *) &data[0];

    response.value = 0;
    response.status = STATUS_INVALID_REQUEST;

    if (rq->bRequest == REQ_ENABLE_LED) {
        led_on();
        response.status = STATUS_OK;

    } else if (rq->bRequest == REQ_DISABLE_LED) {
        led_off();
        response.status = STATUS_OK;

    } else if (rq->bRequest == REQ_SET_FREQ) {
        uint16_t req_index = rq->wIndex.word;
        uint16_t req_value = rq->wValue.word;

        uint8_t output = req_index >> 12;
        uint32_t freq = (((uint32_t)req_index << 16) | req_value) & 0x0FFFFFFF;

        if (output >= CLK_OUTPUTS) {
            response.status = STATUS_INVALID_INDEX;
        } else if (freq > 160000000UL) {
            response.status = STATUS_INVALID_VALUE;
        } else {
            beacon_config.output[output].frequency = freq;
            printf("Setting f_%d to %ld Hz\n", output, freq);
            response.status = STATUS_OK;
        }

    } else if (rq->bRequest == REQ_SET_DRIVE) {
        uint16_t output = rq->wIndex.word;
        uint16_t drive = rq->wValue.word;

        if (output >= CLK_OUTPUTS) {
            response.status = STATUS_INVALID_INDEX;
        } else if (drive & 0x01 || drive > 8) {
            response.status = STATUS_INVALID_VALUE;
        } else {
            beacon_config.output[output].drive = drive;
            printf("Setting drive strength of output %d to %d mA\n", output, drive);
            response.status = STATUS_OK;

            /* Write to the Si5351 */
            si5351_drive_strength(output, (drive / 2) - 1);
        }

    } else if (rq->bRequest == REQ_SET_CALL) {
        uint16_t index = rq->wIndex.word;
        uint16_t value = rq->wValue.word;

        // TODO: Check value of character
        if (index >= CALL_LENGTH) {
            response.status = STATUS_INVALID_INDEX;
        } else {
            beacon_config.callsign[index] = value;
            response.status = STATUS_OK;
            printf("Setting callsign[%d] = '%c'\n", index, value);
        }

    } else if (rq->bRequest == REQ_SET_FREQ_CORR) {
        uint16_t index = rq->wIndex.word;
        uint16_t value = rq->wValue.word;

        int32_t pptm = ((uint32_t)index << 16) | value;
        beacon_config.freq_corr = pptm;
        response.status = STATUS_OK;

        /* Write to the Si5351 */
        si5351_set_correction(pptm);

    } else if (rq->bRequest == REQ_START_TONE) {
        uint16_t out = rq->wIndex.word;

        if (out >= CLK_OUTPUTS) {
            response.status = STATUS_INVALID_INDEX;
        } else {
            si5351_set_freq(beacon_config.output[out].frequency, out);
            si5351_output_enable(out, true);
            response.status = STATUS_OK;
        }

    } else if (rq->bRequest == REQ_STOP_TONE) {
        uint16_t out = rq->wIndex.word;

        if (out >= CLK_OUTPUTS) {
            response.status = STATUS_INVALID_INDEX;
        } else {
            si5351_output_enable(out, false);
            response.status = STATUS_OK;
        }
    }

    usbMsgPtr = (void *) &response;
    return sizeof(response);
}


static int uart_putchar(char c, FILE *stream) {
    uart_putc(c);
}

int main(int argc, char **argv) {
    led_init();

    /* Initialize UART and redirect stdout */
    uart_init(UART_BAUD_SELECT(115200, F_CPU));
    static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &mystdout;

    /* Initialize the USB driver and force enumeration. */
    usbInit();
    usbDeviceDisconnect();
    _delay_ms(250);
    usbDeviceConnect();

    sei();

    /* Initialize I2C and the Si5251 module */
    i2c_init();
    si5351_init(SI5351_CRYSTAL_LOAD_8PF, SI5351_CLK_SRC_XTAL);
    si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);

    printf("WSPR Beacon by OE5TKM\n");

    while (1) {
        usbPoll();
    }

    return 0;
}
