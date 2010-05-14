/*
 * Copyright(C) 2008 Stefan Siegl <stesie@brokenpipe.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 */

#ifndef PINCONFIG_H
#define PINCONFIG_H

#define ZBUS_TX_DDR DDRE
#define ZBUS_TX_PORT PORTE
#define ZBUS_TX_PIN 4

/* If you don't want to have status LEDs, simply comment out these */
//#define STATUS_LED_PORT   PORTB
//#define STATUS_LED_DDR    DDRB
//#define STATUS_LED_TX     PB6
//#define STATUS_LED_RX     PB7

#endif

