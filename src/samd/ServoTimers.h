/*
  Copyright (c) 2015 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

/*
 * Defines for 16 bit timers used with  Servo library
 *
 * If _useTimerX is defined then TimerX is a 16 bit timer on the current board
 * timer16_Sequence_t enumerates the sequence that the timers should be allocated
 * _Nbr_16timers indicates how many 16 bit timers are available.
 */

#ifndef __SERVO_TIMERS_H__
#define __SERVO_TIMERS_H__


/**
 * SAMD Only definitions
 * ---------------------
 */

/*
 * Make sure that macros __SAMD21__ and __SAMD51__ are defined.
 * Use CMSIS definitions to figure out on which processor we run.
 */

// Definitions for processors taken from here:
// https://github.com/arduino/ArduinoModule-CMSIS-Atmel/blob/master/CMSIS-Atmel/CMSIS/Device/ATMEL/samd21/include/samd21.h
// This list contains most of the SAMD21 processors.
// Not included are the B, C, L, and D device variants listed in the data sheet.
#if !defined(__SAMD21__)
  #if defined(__SAMD21E15A__)  || defined(__ATSAMD21E15A__)  \
  ||  defined(__SAMD21E16A__)  || defined(__ATSAMD21E16A__)  \
  ||  defined(__SAMD21E17A__)  || defined(__ATSAMD21E17A__)  \
  ||  defined(__SAMD21E18A__)  || defined(__ATSAMD21E18A__)  \
  ||  defined(__SAMD21G15A__)  || defined(__ATSAMD21G15A__)  \
  ||  defined(__SAMD21G16A__)  || defined(__ATSAMD21G16A__)  \
  ||  defined(__SAMD21G17A__)  || defined(__ATSAMD21G17A__)  \
  ||  defined(__SAMD21G17AU__) || defined(__ATSAMD21G17AU__) \
  ||  defined(__SAMD21G18A__)  || defined(__ATSAMD21G18A__)  \
  ||  defined(__SAMD21G18AU__) || defined(__ATSAMD21G18AU__) \
  ||  defined(__SAMD21J15A__)  || defined(__ATSAMD21J15A__)  \
  ||  defined(__SAMD21J16A__)  || defined(__ATSAMD21J16A__)  \
  ||  defined(__SAMD21J16AC__) || defined(__ATSAMD21J16AC__) \
  ||  defined(__SAMD21J17A__)  || defined(__ATSAMD21J17A__)  \
  ||  defined(__SAMD21J17AC__) || defined(__ATSAMD21J17AC__) \
  ||  defined(__SAMD21J18A__)  || defined(__ATSAMD21J18A__)  \
  ||  defined(__SAMD21J18AC__) || defined(__ATSAMD21J18AC__)
    #define __SAMD21__
  #endif
#endif

// Definitions for processors taken from here: 
// https://github.com/arduino/ArduinoModule-CMSIS-Atmel/blob/master/CMSIS-Atmel/CMSIS/Device/ATMEL/samd51/include/samd51.h
// The list contains all SAMD51 processors (according to data sheet as of April 2021).
#if !defined(__SAMD51__)
  #if defined(__SAMD51G18A__) || defined(__ATSAMD51G18A__) \
  ||  defined(__SAMD51G19A__) || defined(__ATSAMD51G19A__) \
  ||  defined(__SAMD51J18A__) || defined(__ATSAMD51J18A__) \
  ||  defined(__SAMD51J19A__) || defined(__ATSAMD51J19A__) \
  ||  defined(__SAMD51J20A__) || defined(__ATSAMD51J20A__) \
  ||  defined(__SAMD51N19A__) || defined(__ATSAMD51N19A__) \
  ||  defined(__SAMD51N20A__) || defined(__ATSAMD51N20A__) \
  ||  defined(__SAMD51P19A__) || defined(__ATSAMD51P19A__) \
  ||  defined(__SAMD51P20A__) || defined(__ATSAMD51P20A__)
    #define __SAMD51__
  #endif
#endif

/*
 * Use different TC instances for SAMD21 and SAMD51.
* 
 * For SAMD21, we use TC4 and TC5 as timers.
 * For SAMD51, we use TC0 and TC1 as timers.
 * Those TC instances are guaranteed to exist on all SAMD21 and SAMD51
 * processors, respectively.
 * 
 * TC4/TC5 on SAMD21 and TC0/TC1 on SAMD51,respectively, use the same 
 * peripheral clock. If we configure the clock for one of these TC instances,
 * we *will* also configure the clock for the other. This means that 
 * stuff on *both* TC instances will probably break when using this
 * library (most notably PWM with analogWrite()).
 */

#define _useTimer1
#define _useTimer2

#if defined(__SAMD21__)
    #if defined (_useTimer1)
        #define TIMER1_TC                 TC4          // TC instance to use. Pointer of type Tc.
        #define TIMER1_IRQn               TC4_IRQn     // Interrupt line for this TC instance.
        #define TIMER1_HANDLER            TC4_Handler  // ISR for this TC instance
        #define TIMER1_GCLK_ID            TC4_GCLK_ID  // index of peripheral for GCLK
    #endif
    #if defined (_useTimer2)
        #define TIMER2_TC                 TC5          // TC instance to use. Pointer of type Tc.
        #define TIMER2_IRQn               TC5_IRQn     // Interrupt line for this TC instance.
        #define TIMER2_HANDLER            TC5_Handler  // ISR for this TC instance
        #define TIMER2_GCLK_ID            TC5_GCLK_ID  // index of peripheral for GCLK
    #endif
#elif defined(__SAMD51__) 
    #if defined (_useTimer1)
        #define TIMER1_TC                 TC0          // TC instance to use. Pointer of type Tc.
        #define TIMER1_IRQn               TC0_IRQn     // Interrupt line for this TC instance.
        #define TIMER1_HANDLER            TC0_Handler  // ISR for this TC instance
        #define TIMER1_GCLK_ID            TC0_GCLK_ID  // index of peripheral for GCLK
    #endif  
    #if defined (_useTimer2)
        #define TIMER2_TC                 TC1          // TC instance to use. Pointer of type Tc.
        #define TIMER2_IRQn               TC1_IRQn     // Interrupt line for this TC instance.
        #define TIMER2_HANDLER            TC1_Handler  // ISR for this TC instance
        #define TIMER2_GCLK_ID            TC1_GCLK_ID  // index of peripheral for GCLK
    #endif
#endif

typedef enum {
#if defined (_useTimer1)
    _timer1,
#endif
#if defined (_useTimer2)
    _timer2,
#endif
    _Nbr_16timers } timer16_Sequence_t;

typedef enum {
    _cc0,
    _cc1,
    _Nbr_CC_registers } CC_register_t;

#endif   // __SERVO_TIMERS_H__
