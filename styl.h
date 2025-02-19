/*
 * styl - Nano Stylophone for attiny13a
 * Copyright (C) 2011  Bob Clough <bob@clough.me>
 * Copyright (C) 2011  Charles Yarnold <charlesyarnold@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define TRUE 1
#define FALSE 0

#define LED_REC BIT(2)
#define LED_TOUCH BIT(3)

void playnote(uint8_t);
void dontplay(void);

void pwm_init(void);

void adc_init(void);
int adc_read(void);





