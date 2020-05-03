/*
 Servo.cpp - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
 Copyright (c) 2009 Michael Margolis.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#if defined(ARDUINO_ARCH_AVR)

#include <Arduino.h>
#include <avr/interrupt.h>

#include "Servo.h"

#define PRESCALE 8

#define usToTicks(_us) ((clockCyclesPerMicrosecond() * _us) / PRESCALE)                 // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (((unsigned)_ticks * PRESCALE) / clockCyclesPerMicrosecond()) // converts from ticks back to microseconds

#define TRIM_DURATION 2 // compensation ticks to trim adjust for digitalWrite delays // 12 August 2009

static servo_t servos[MAX_SERVOS] = {0};             // static array of servo structures
static volatile int8_t channel[_Nbr_16timers] = {0}; // counter for the servo being pulsed for each timer (or -1 if refresh interval)

static volatile uint8_t servoCount = 0; // the total number of attached servos

// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER)) // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)                     // returns the globalIndex of the servo on this timer
#define SERVO_INDEX(_timer, _channel) ((_timer * SERVOS_PER_TIMER) + _channel)                 // macro to access servo globalIndex by timer and channel
#define SERVO(_timer, _channel) (servos[SERVO_INDEX(_timer, _channel)])                        // macro to access servo class by timer and channel
#define MIN(x, y) (x) > (y) ? (y) : (x)

#define DURATION_DECREMENT (REFRESH_INTERVAL / 1000)

// Pin makros
//////////////////////////////////////////
// macros from http://masteringarduino.blogspot.com.es/2013/10/fastest-and-smallest-digitalread-and.html
/////////////////////////////////////////
#define portOfPin(P) \
    (((P) >= 0 && (P) < 8) ? &PORTD : (((P) > 7 && (P) < 14) ? &PORTB : &PORTC))
#define ddrOfPin(P) \
    (((P) >= 0 && (P) < 8) ? &DDRD : (((P) > 7 && (P) < 14) ? &DDRB : &DDRC))
#define pinOfPin(P) \
    (((P) >= 0 && (P) < 8) ? &PIND : (((P) > 7 && (P) < 14) ? &PINB : &PINC))
#define pinIndex(P) ((uint8_t)(P > 13 ? P - 14 : P & 7))
#define pinMask(P) ((uint8_t)(1 << pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P)) &= ~pinMask(P)
#define pinAsInputPullUp(P)        \
    *(ddrOfPin(P)) &= ~pinMask(P); \
    digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P)) |= pinMask(P)
#define digitalLow(P) *(portOfPin(P)) &= ~pinMask(P)
#define digitalHigh(P) *(portOfPin(P)) |= pinMask(P)
#define isHigh(P) ((*(pinOfPin(P)) & pinMask(P)) > 0)
#define isLow(P) ((*(pinOfPin(P)) & pinMask(P)) == 0)
#define digitalState(P) ((uint8_t)isHigh(P))

#define digitalToggle(P) *(portOfPin(P)) ^= pinMask(P)

/************ static functions common to all instances ***********************/

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t *OCRnA) {
    uint8_t globalIndex = SERVOS_PER_TIMER * timer;
    uint8_t servoCountOfTimer = servoCount - globalIndex;
    if (servoCountOfTimer > SERVOS_PER_TIMER) {
        servoCountOfTimer = SERVOS_PER_TIMER;
    }
    globalIndex += channel[timer];

    // increment to the next channel
    if (channel[timer] < 0) {
        *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer
    } else if (channel[timer] < servoCountOfTimer) {
        const servo_t &current = servos[globalIndex];
        if (current.Pin.isActive) {
            // pulse this channel low if activated
            *current.Pin.reg &= ~current.Pin.mask;
        }
    }

    ++channel[timer];
    ++globalIndex;

    if (channel[timer] < servoCountOfTimer) {
        servo_t &nextServo = servos[globalIndex];

        // Setup the compare register to the wanted time to pulse back to low
        *OCRnA = *TCNTn + nextServo.ticks;

        // check if activated and has time left
        if (nextServo.Pin.isActive && nextServo.duration > 0) {
            // its an active channel so pulse it high
            *nextServo.Pin.reg |= nextServo.Pin.mask;
            if (nextServo.duration != 0xFFFF)
                --nextServo.duration;
        }
    } else {
        // start again at the first channel
        channel[timer] = -1;
        // finished all channels so wait for the refresh period to expire before starting over

        // allow a few ticks to ensure the next OCR1A won't be missed
        uint16_t ticks = *TCNTn + 4;
        if (ticks < usToTicks(REFRESH_INTERVAL)) {
            *OCRnA = (uint16_t)usToTicks(REFRESH_INTERVAL);
        } else {
            // at least REFRESH_INTERVAL has elapsed we need a interrupt ASAP
            *OCRnA = ticks;
        }
    }
}

#ifndef WIRING // Wiring pre-defines signal handlers so don't define any if compiling for the Wiring platform
// Interrupt handlers for Arduino
#if defined(_useTimer1)
SIGNAL(TIMER1_COMPA_vect) {
    handle_interrupts(_timer1, &TCNT1, &OCR1A);
}
#endif

#if defined(_useTimer3)
SIGNAL(TIMER3_COMPA_vect) {
    handle_interrupts(_timer3, &TCNT3, &OCR3A);
}
#endif

#if defined(_useTimer4)
SIGNAL(TIMER4_COMPA_vect) {
    handle_interrupts(_timer4, &TCNT4, &OCR4A);
}
#endif

#if defined(_useTimer5)
SIGNAL(TIMER5_COMPA_vect) {
    handle_interrupts(_timer5, &TCNT5, &OCR5A);
}
#endif

#elif defined(WIRING)
// Interrupt handlers for Wiring
#if defined(_useTimer1)
void Timer1Service() {
    handle_interrupts(_timer1, &TCNT1, &OCR1A);
}
#endif
#if defined(_useTimer3)
void Timer3Service() {
    handle_interrupts(_timer3, &TCNT3, &OCR3A);
}
#endif
#endif

static void initISR(timer16_Sequence_t timer) {
#if defined(_useTimer1)
    if (timer == _timer1) {
        TCCR1A = 0;         // normal counting mode
        TCCR1B = _BV(CS11); // set prescaler of 8
        TCNT1 = 0;          // clear the timer count
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega128__)
        TIFR |= _BV(OCF1A);   // clear any pending interrupts;
        TIMSK |= _BV(OCIE1A); // enable the output compare interrupt
#else
        // here if not ATmega8 or ATmega128
        TIFR1 |= _BV(OCF1A);   // clear any pending interrupts;
        TIMSK1 |= _BV(OCIE1A); // enable the output compare interrupt
#endif
#if defined(WIRING)
        timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service);
#endif
    }
#endif

#if defined(_useTimer3)
    if (timer == _timer3) {
        TCCR3A = 0;         // normal counting mode
        TCCR3B = _BV(CS31); // set prescaler of 8
        TCNT3 = 0;          // clear the timer count
#if defined(__AVR_ATmega128__)
        TIFR |= _BV(OCF3A);    // clear any pending interrupts;
        ETIMSK |= _BV(OCIE3A); // enable the output compare interrupt
#else
        TIFR3 = _BV(OCF3A);    // clear any pending interrupts;
        TIMSK3 = _BV(OCIE3A);  // enable the output compare interrupt
#endif
#if defined(WIRING)
        timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service); // for Wiring platform only
#endif
    }
#endif

#if defined(_useTimer4)
    if (timer == _timer4) {
        TCCR4A = 0;           // normal counting mode
        TCCR4B = _BV(CS41);   // set prescaler of 8
        TCNT4 = 0;            // clear the timer count
        TIFR4 = _BV(OCF4A);   // clear any pending interrupts;
        TIMSK4 = _BV(OCIE4A); // enable the output compare interrupt
    }
#endif

#if defined(_useTimer5)
    if (timer == _timer5) {
        TCCR5A = 0;           // normal counting mode
        TCCR5B = _BV(CS51);   // set prescaler of 8
        TCNT5 = 0;            // clear the timer count
        TIFR5 = _BV(OCF5A);   // clear any pending interrupts;
        TIMSK5 = _BV(OCIE5A); // enable the output compare interrupt
    }
#endif
}

static void finISR(timer16_Sequence_t timer) {
    //disable use of the given timer
#if defined(_useTimer1)
    if (timer == _timer1) {
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega128__)
        TIMSK &= ~_BV(OCIE1A); // disable timer 1 output compare interrupt
#else
        // here if not ATmega8 or ATmega128
        TIMSK1 &= ~_BV(OCIE1A); // disable timer 1 output compare interrupt
#endif
#if defined(WIRING) // Wiring
        timerDetach(TIMER1OUTCOMPAREA_INT);
#endif
    }
#endif

#if defined(_useTimer3)
    if (timer == _timer3) {
#if defined(__AVR_ATmega128__)
        ETIMSK &= ~_BV(OCIE3A); // disable the timer3 output compare A interrupt
#else
        TIMSK3 &= ~_BV(OCIE3A); // disable the timer3 output compare A interrupt
#endif
#if defined(WIRING) // Wiring
        timerDetach(TIMER3OUTCOMPAREA_INT);
#endif
    }
#endif
}

static inline bool isTimerRunning(timer16_Sequence_t timer) {

    //disable use of the given timer
#if defined(_useTimer1)
    if (timer == _timer1) {
        // check if timer 1 output compare interrupt flag is set
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega128__)
        return TIMSK & _BV(OCIE1A);
#else
        // here if not ATmega8 or ATmega128
        return TIMSK1 & _BV(OCIE1A);
#endif
    }
#endif

#if defined(_useTimer3)
    if (timer == _timer3) {
        // check if timer 3 output compare interrupt flag is set
#if defined(__AVR_ATmega128__)
        return ETIMSK & _BV(OCIE3A);
#else
        return TIMSK3 & _BV(OCIE3A);
#endif
    }
#endif

    return false;
}

static inline bool isTimerUsed(timer16_Sequence_t timer) {
    uint8_t globalIndex = SERVOS_PER_TIMER * timer;
    uint8_t servoCountOfTimer = servoCount - globalIndex;
    if (servoCountOfTimer > SERVOS_PER_TIMER) {
        servoCountOfTimer = SERVOS_PER_TIMER;
    }
    // returns true if any servo is active on this timer
    for (uint8_t channel = 0; channel < servoCountOfTimer; ++channel) {
        servo_t &current = servos[globalIndex++];
        if (current.Pin.isActive && current.duration > 0)
            return true;
    }
    return false;
}

/****************** end of static functions ******************************/

Servo::Servo() : servoIndex(servoCount < MAX_SERVOS ? servoCount++ : INVALID_SERVO) {
    if (servoIndex != INVALID_SERVO) {
        servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH); // store default values  - 12 Aug 2009
    }
}

uint8_t Servo::attach(uint8_t pin) {
    return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(uint8_t pin, uint16_t min, uint16_t max) {
    if (this->servoIndex != INVALID_SERVO) {
        servo_t &servo = servos[this->servoIndex];

        pinAsOutput(pin); // set servo pin to output
        servo.Pin.reg = portOfPin(pin);
        servo.Pin.mask = pinMask(pin);

        // Ensure the pin is set to low
        *servo.Pin.reg &= ~servo.Pin.mask;

        // Ensure the servo will not be controlled before setting target value
        servo.duration = 0;
        servo.Pin.isActive = true; // this must be set after the check for isTimerUsed

        this->min = min;
        this->max = max;
    }
    return this->servoIndex;
}

void Servo::detach() {
    servo_t &servo = servos[this->servoIndex];
    servo.Pin.isActive = false;
    // Ensure signal is set to LOW
    *servo.Pin.reg &= ~servo.Pin.mask;
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if (!isTimerUsed(timer)) {
        finISR(timer);
    }
}

void Servo::write(uint16_t value, uint16_t duration) {
    if (value < this->min) { // treat values less than the minimum pulse width as angles in degrees (valid values in microseconds are handled as microseconds)
        value = map(value, 0, 180, this->min, this->max);
    }
    this->writeMicroseconds(value, duration);
}

void Servo::writeMicroseconds(uint16_t value, uint16_t duration) {
    // calculate and store the values for the given channel
    const uint8_t channel = this->servoIndex;
    // ensure channel is valid
    if (channel != INVALID_SERVO) {
        value = constrain(value, this->min, this->max); // ensure pulse width is valid
        value = value - TRIM_DURATION;
        value = usToTicks(value); // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

        servo_t &servo = servos[channel];

        // ceil if duration != 0 at least one iteration
        uint16_t neededIterations = (duration + DURATION_DECREMENT - 1) / DURATION_DECREMENT;

        uint8_t oldSREG = SREG;
        cli();
        servo.ticks = value;
        // Set duration
        servo.duration = duration ? neededIterations : 0xFFFF;
        SREG = oldSREG;

        // initialize the timer if it has not already been initialized
        timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(this->servoIndex);
        if (!isTimerRunning(timer)) {
            initISR(timer);
        }
    }
}

// return the value as degrees
uint8_t Servo::read() {
    return map(this->readMicroseconds() + 1, this->min, this->max, 0, 180);
}

uint16_t Servo::readMicroseconds() {
    if (this->servoIndex != INVALID_SERVO) {
        return ticksToUs(servos[this->servoIndex].ticks) + TRIM_DURATION; // 12 aug 2009
    } else {
        return 0;
    }
}

bool Servo::attached() {
    return servos[this->servoIndex].Pin.isActive;
}

#endif // ARDUINO_ARCH_AVR
