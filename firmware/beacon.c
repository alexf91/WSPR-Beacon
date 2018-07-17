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
#include <util/delay.h>

#include "uart.h"
#include "config.h"
#include "usbdrv.h"


static inline void led_init(void) { DDRB |= (1<<PB5); }
static inline void led_on(void) { PORTB |= (1<<PB5); }
static inline void led_off(void) { PORTB &= ~(1<<PB5); }
static inline void led_toggle(void) { PORTB ^= (1<<PB5); }


usbMsgLen_t usbFunctionSetup(uchar setupData[8])
{
    return 0;
}


int main(int argc, char **argv) {
    led_init();
    uart_init(UART_BAUD_SELECT(115200, F_CPU));
    uart_puts("WSPR Beacon by OE5TKM\n");

    /* Initialize the USB driver and force enumeration. */
    usbInit();
    usbDeviceDisconnect();
    _delay_ms(500);
    usbDeviceConnect();

    sei();

    while (1) {
        usbPoll();
    }

    return 0;
}
