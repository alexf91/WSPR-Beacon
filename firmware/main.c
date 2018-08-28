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
#include <util/atomic.h>

#include "uart.h"
#include "usbdrv.h"
#include "beacon.h"
#include "debug.h"


/* USB requests */
enum Request {
    REQ_ENABLE_LED = 0,     /* Enable the LED */
    REQ_DISABLE_LED,        /* Disable the LED */
    REQ_SET_FREQ,           /* Set the frequency of an output */
    REQ_SET_DRIVE,          /* Set drive strenght for an output */
    REQ_SET_CALL,           /* Set the callsign */
    REQ_SET_FREQ_CORR,      /* Set frequency correction value */
    REQ_TRANSMIT_TONE,      /* Transmit a continuous tone */
    REQ_TRANSMIT_WSPR,      /* Start a WSPR transmission */
    REQ_SET_LAT,            /* Set the latitude */
    REQ_SET_LON,            /* Set the longitude */
};


static inline void led_init(void) { DDRB |= (1<<PB5); }
static inline void led_on(void) { PORTB |= (1<<PB5); }
static inline void led_off(void) { PORTB &= ~(1<<PB5); }


/* Structure used as response to requests. */
struct {
    uint16_t value;
    uint16_t status;
} response;


usbMsgLen_t usbFunctionSetup(uchar data[8]) {
    usbRequest_t *rq = (void *) &data[0];

    response.value = 0;
    response.status = 128;

    if (rq->bRequest == REQ_ENABLE_LED) {
        led_on();
        response.status = 0;

    } else if (rq->bRequest == REQ_DISABLE_LED) {
        led_off();
        response.status = 0;

    } else if (rq->bRequest == REQ_SET_FREQ) {
        uint16_t req_index = rq->wIndex.word;
        uint16_t req_value = rq->wValue.word;

        uint8_t output = req_index >> 12;
        uint32_t freq = (((uint32_t)req_index << 16) | req_value) & 0x0FFFFFFF;

        response.status = beacon_set_freq(output, freq);

    } else if (rq->bRequest == REQ_SET_DRIVE) {
        uint16_t output = rq->wIndex.word;
        uint16_t drive = rq->wValue.word;

        response.status = beacon_set_drive_strength(output, drive);

    } else if (rq->bRequest == REQ_SET_CALL) {
        uint16_t index = rq->wIndex.word;
        uint16_t value = rq->wValue.word;

        response.status = beacon_set_callsign(index, value);

    } else if (rq->bRequest == REQ_SET_FREQ_CORR) {
        uint16_t index = rq->wIndex.word;
        uint16_t value = rq->wValue.word;
        int32_t pptm = ((uint32_t)index << 16) | value;

        response.status = beacon_set_correction(pptm);

    } else if (rq->bRequest == REQ_TRANSMIT_TONE) {
        uint16_t out = rq->wIndex.word;
        uint16_t msecs = rq->wValue.word;

        response.status = beacon_transmit_tone(out, msecs);

    } else if (rq->bRequest == REQ_TRANSMIT_WSPR) {
        uint16_t out = rq->wIndex.word;
        response.status = beacon_transmit_wspr(out);

    } else if (rq->bRequest == REQ_SET_LON) {
        uint16_t req_index = rq->wIndex.word;
        uint16_t req_value = rq->wValue.word;

        uint32_t lon = ((uint32_t)req_index << 16) | req_value;

        response.status = beacon_set_longitude(lon);

    } else if (rq->bRequest == REQ_SET_LAT) {
        uint16_t req_index = rq->wIndex.word;
        uint16_t req_value = rq->wValue.word;

        uint32_t lat = ((uint32_t)req_index << 16) | req_value;

        response.status = beacon_set_latitude(lat);
    }

    usbMsgPtr = (void *) &response;
    return sizeof(response);
}


/**
 * Wrapper for stdout redirection.
 */
static int uart_putchar(char c, FILE *stream) {
    uart_putc(c);
    return 0;
}


int main(int argc, char **argv) {
    led_init();

    /* Initialize UART and redirect stdout */
    uart_init(UART_BAUD_SELECT(115200, F_CPU));
    static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &mystdout;

    /* Initialize the beacon. This initializes I2C, Timer 0 and the Si5351. */
    if (beacon_init()) {
        debug("Beacon initialization failed\n");
        while (1);
    }

    /* Initialize the USB driver and force enumeration. */
    usbInit();
    usbDeviceDisconnect();
    _delay_ms(250);
    usbDeviceConnect();

    sei();

    debug("WSPR Beacon by OE5TKM\n");

    while (1) {
        usbPoll();
        beacon_poll();
    }
    return 0;
}
