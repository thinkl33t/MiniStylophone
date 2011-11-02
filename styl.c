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

#define F_CPU 8000000UL
#define USART_BAUDRATE 9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include "macros.h"
#include "notes.h"
#include "styl.h"

int recording, saved, playing, reading, noteSlot;

void setup(void)
{
    // Set PORTB pins 2 and 3 as outputs
    bit_set(DDRB, LED_TOUCH);
    bit_set(DDRB, LED_REC);

    // Turn Touch LED on, Record LED off
    bit_clear(PORTB, LED_TOUCH);
    bit_set(PORTB, LED_REC);

    // Activate PWM for output, ADC for probe input
    pwm_init();
    adc_init();

    // We aren't recording
    recording = FALSE;

    // We are at the beginning of a recording
    noteSlot = 0;
}

void loop(void)
{
    //if we are in playback mode
    if (playing)
    {
        // fetch the next note from the eeprom
        reading = eeprom_read_byte((uint8_t*) noteSlot);
        reading = reading * 4;
    }
    else
    {
        // fetch the next note from the probe
        reading = adc_read();
    }

    // if nothing is being pressed
    if (reading < 128)
    {
        // turn the touch LED On
        bit_clear(PORTB, LED_TOUCH);

        // Stop playing any notes we might be playing
        dontplay();

        // if we are recording
        if (recording)
        {
            // turn the REC LED on
            bit_clear(PORTB, LED_REC);
            _delay_ms(10);
            if (adc_read() == reading)
            {
                saved = 0;
            }
        }
    }
    else
    {
        // if we are recording and the note hasn't changed
        if (recording && !saved)
        {
            // crop off the top two bits to make the note fit in an 8 bit memory address
            int readingLess = reading / 4;

            // store the note in the eeprom
            eeprom_write_byte((uint8_t *) noteSlot, (uint8_t) readingLess);

            // move to the next eeprom location
            noteSlot++;

            // if the eeprom is full
            if (noteSlot > 60)
            {
                // stop recording & reset to the beginning of the recording
                recording = FALSE;
                noteSlot = 0;

                // turn the recording LED Off
                bit_set(PORTB, LED_REC);
            }
            else
            {
                // mark the current note as saved
                saved = 1;
            }
        }

        // turn the TOUCH LED off
        bit_set(PORTB, LED_TOUCH);

        // if we are recording, turn the RED LED off (why here?)
        if (recording) bit_set(PORTB, LED_REC);

        // average out the reading over a few samples to make sure it is correct
        _delay_ms(50);
        reading = reading + adc_read() + adc_read() + adc_read() / 4;

        if ((reading <= 310)) // Record button was pressed
        {
            // Toggle Recording Mode
            recording = !recording;

            // If we are finished recording
            if (!recording)
            {
                // add a '0' to the end of the recording, to make sure the playback is turned off
                noteSlot++;
                eeprom_write_byte((uint8_t *) noteSlot, (uint8_t) 0);
            }

            // reset our 'pointer' to the beginning of the eeprom
            noteSlot = 0;

            // wait for the button to be released
            while (adc_read() > 128);
        }
        else if ((reading <= 340)) // Play button was pressed
        {
            // we cant play while recording
            if (!recording)
            {
                // start playing back from the beginning of the eeprom
                playing = TRUE;
                noteSlot = 0;

                // turn on the REC LED
                bit_clear(PORTB, LED_REC);

                // wait for the button to be released
                while (adc_read() > 128);
            }
        }
        else if ((reading <= 359))
        {
            playnote(A1);
        }
        else if (reading <= 380)
        {
            playnote(AS1);
        }
        else if (reading <= 387)
        {
            playnote(B1);
        }
        else if (reading <= 402)
        {
            playnote(C1);
        }
        else if (reading <= 418)
        {
            playnote(CS1);
        }
        else if (reading <= 436)
        {
            playnote(D1);
        }
        else if (reading <= 455)
        {
            playnote(DS1);
        }
        else if (reading <= 477)
        {
            playnote(E1);
        }
        else if (reading <= 500)
        {
            playnote(F1);
        }
        else if (reading <= 525)
        {
            playnote(FS1);
        }
        else if (reading <= 554)
        {
            playnote(G1);
        }
        else if (reading <= 586)
        {
            playnote(GS1);
        }
        else if (reading <= 621)
        {
            playnote(A2);
        }
        else if (reading <= 661)
        {
            playnote(AS2);
        }
        else if (reading <= 707)
        {
            playnote(B2);
        }
        else if (reading <= 760)
        {
            playnote(C2);
        }
        else if (reading <= 821)
        {
            playnote(CS2);
        }
        else if (reading <= 892)
        {
            playnote(D2);
        }
        else if (reading <= 977)
        {
            playnote(DS2);
        }
        else if (reading > 977)
        {
            playnote(E2);
        }
    }

    if (playing)
    {
        // wait for time
        _delay_ms(200);
        dontplay();
        _delay_ms(200);
        noteSlot++;
        if (noteSlot > 60 || reading == 0)
        {
            noteSlot = 0;
            playing = FALSE;
            bit_set(PORTB, LED_REC);
        }
    }
    else
    {
        // loop quickly
        _delay_us(255);
    }
}

void playnote(uint8_t a)
{
    OCR0A = a;
    OCR0B = 0;
}

void dontplay()
{
    playnote(0);
}

void pwm_init(void)
{
    DDRB |= BIT(1);
    TCCR0A = BIT(COM0B1); // Clear OC0A on Compare Match
    TCCR0A |= BIT(WGM00); // PWM mode, 8 bit
    TCCR0B = BIT(WGM02); // TOP = OCRA
    TCCR0B |= BIT(CS01) | BIT(CS00); // 64x prescaler
}

void adc_init(void)
{
    ADMUX = BIT(MUX1);
    ADCSRA = BIT(ADEN);
}

int adc_read()
{
    ADCSRA |= BIT(ADSC);
    while (bit_get(ADCSRA, BIT(ADSC)) != 0)
    {
    };
    return ADC;
}

/**
 * Main to allow arduino-style setup and main functions
 */
int main(void)
{
    setup();
    while (1)
    {
        loop();
    }
}