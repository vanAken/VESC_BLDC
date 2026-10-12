/*
	Copyright 2021 Marcos Chaparro	mchaparro@powerdesigns.ca

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef APP_LUNA_DISPLAY_SERIAL_H_
#define APP_LUNA_DISPLAY_SERIAL_H_

#include "stdint.h"
#include "stdbool.h"

void luna_display_serial_start(int8_t initial_level);
// Zeigt Displayverbindung, Stufe und Reku.
void luna_display_serial_print_status(void);
// Display sendet (letztes Byte vor hoechstens 2 s).
bool luna_display_serial_connected(void);
void luna_display_serial_licht_blink(void);
void luna_display_serial_status_setzen(uint8_t code);
uint8_t luna_display_status(void);
void luna_display_serial_null_senden(int sekunden);
void luna_display_serial_tacho_zeigen(int kmh, int sekunden);
void luna_display_serial_licht_signal(bool start);
// Ladestand von Hand vorgeben (0..100) oder -1 fuer die echte Messung.
void luna_display_serial_soc_force(int prozent);
int luna_display_serial_soc_forced(void);

#endif /* APP_LUNA_DISPLAY_SERIAL_H_ */
